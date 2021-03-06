//
// Copyright 2018 Mateusz Polnik
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QUAKE_BASE_MIP_MODEL_H
#define QUAKE_BASE_MIP_MODEL_H

#include <unordered_map>

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/date_time.hpp>

#include <gurobi_c++.h>

#include "util/gurobi.h"

#include "solution.h"
#include "validator.h"

namespace quake {

    class BaseMipModel {
    public:
        explicit BaseMipModel(ExtendedProblem const *problem)
                : problem_{problem},
                  stations_{GroundStation::None},
                  dummy_station_index_{0},
                  mip_environment_{},
                  mip_model_{mip_environment_} {
            const auto &problem_stations = problem_->Stations();
            CHECK(std::find(std::cbegin(problem_stations), std::cend(problem_stations), GroundStation::None) == std::cend(problem_stations));

            std::copy(std::cbegin(problem_stations), std::cend(problem_stations), std::back_inserter(stations_));
            for (std::size_t index_pos = 0; index_pos < stations_.size(); ++index_pos) {
                station_indices_.emplace(stations_.at(index_pos), index_pos);
            }
        }

        boost::optional<Solution> Solve(const boost::optional<boost::posix_time::time_duration> &time_limit_opt,
                                        const boost::optional<double> &gap_opt,
                                        const boost::optional<Solution> &initial_guess) {
            if (time_limit_opt) {
                mip_model_.set(GRB_DoubleParam_TimeLimit, time_limit_opt->total_seconds());
            }

            if (gap_opt) {
                mip_model_.set(GRB_DoubleParam_MIPGap, *gap_opt);
            }

//            mip_model_.set(GRB_IntParam_Presolve, GRB_PRESOLVE_AGGRESSIVE);
//            mip_model_.set(GRB_IntParam_MIPFocus, GRB_MIPFOCUS_BALANCED);
//            mip_model_.set(GRB_IntParam_Cuts, GRB_CUTS_AUTO);

            try {
                Build(initial_guess);
                mip_model_.optimize();
            } catch (const GRBException &exception) {
                LOG(FATAL) << "Solver exception " << exception.getMessage() << " error code: " << exception.getErrorCode();
            }

            using namespace util;

            const auto solver_status_code = mip_model_.get(GRB_IntAttr_Status);
            const auto solver_status = static_cast<SolverStatus>(solver_status_code);
            CHECK_NE(solver_status, SolverStatus::Loaded);
            CHECK_NE(solver_status, SolverStatus::InProgress);
            CHECK_NE(solver_status, SolverStatus::Numeric);

//            if (solver_status == SolverStatus::Unbounded) {
//                if (mip_model_.get(GRB_IntParam_InfUnbdInfo) == 1) {
//                    const auto num_vars = mip_model_.get(GRB_IntAttr_NumVars);
//                    for (auto variable_index = 0; variable_index < num_vars; ++variable_index) {
//                        try {
//                            const auto ray_coefficient = mip_model_.getVar(variable_index).get(GRB_DoubleAttr_UnbdRay);
//                            if (ray_coefficient != 0) {
//                                LOG(INFO) << ray_coefficient << mip_model_.getVar(variable_index).get(GRB_StringAttr_VarName);
//                            }
//                        }
//                        catch (const GRBException &ex) {
//                            if (ex.getErrorCode() == 10005) {
//                                continue;
//                            }
//                        }
//                    }
//                }
//            }

            CHECK_NE(solver_status, SolverStatus::Unbounded);
            CHECK_NE(solver_status, SolverStatus::InfiniteOrUnbounded);
            CHECK_NE(solver_status, SolverStatus::Infeasible);
            ReportResults(solver_status);

            const auto num_solutions = mip_model_.get(GRB_IntAttr_SolCount);
            if (num_solutions < 1) {
                return boost::none;
            }

            auto observations = GetObservations();
            std::unordered_map<quake::GroundStation, int64> final_buffers;
            for (const auto &station: Stations()) {
                if (station == GroundStation::None) { continue; }

                final_buffers.emplace(station, 0);
            }

            Metadata metadata;
            {
                static const std::vector<Metadata::Property> PROPERTIES_TO_COPY{
                        Metadata::Property::SwitchDuration,
                        Metadata::Property::ObservationPeriod,
                        Metadata::Property::ScenarioGenerator
                };

                const auto &problem_metadata = problem_->GetMetadata();

                for (const auto property: PROPERTIES_TO_COPY) {
                    auto json_object_opt = problem_metadata.GetProperty<nlohmann::json>(property);
                    if (json_object_opt) {
                        metadata.SetProperty(property, *json_object_opt);
                    }
                }

                if (gap_opt) {
                    metadata.SetProperty(Metadata::Property::GapLimit, *gap_opt);
                }

                if (time_limit_opt) {
                    metadata.SetProperty(Metadata::Property::TimeLimit, *time_limit_opt);
                }

                auto gap = 0.0;
                const auto objectives_number = mip_model_.get(GRB_IntParam_ObjNumber);
                if (objectives_number == 0) {
                    // the default single objective case
                    gap = mip_model_.get(GRB_DoubleAttr_MIPGap);
                } else {
                    const auto obj_env = mip_model_.getMultiobjEnv(0);
                    gap = obj_env.get(GRB_DoubleParam_MIPGap);
                }
                metadata.SetProperty(Metadata::Property::Gap, gap);
            }

            AppendMetadata(metadata);

            Solution solution{std::move(metadata), std::move(observations), std::move(final_buffers)};

            Validator validator{*problem_};
            validator.Validate(solution);

            return boost::make_optional(solution);
        }

        Solution UpdateFinalBuffers(const Solution &solution, const Forecast &forecast) const {
            std::unordered_map<quake::GroundStation, int64> final_buffers;
            for (const auto &station: solution.Stations()) {
                if (station == GroundStation::None) { continue; }

                // number of keys transferred
                double keys_received = 0.0;
                for (const auto &observation_window : solution.ObservationWindows(station)) {
                    keys_received += problem_->KeyRate(station, observation_window, forecast);
                }

                // keys consumed
                const auto total_days = problem_->ObservationPeriod().length().total_seconds() / boost::posix_time::hours(24).total_seconds();
                const auto total_key_consumption = problem_->KeyConsumption(station) * total_days;

                // initial buffer
                const auto station_initial_buffer = problem_->InitialBuffer(station);

                CHECK_GE(station_initial_buffer - total_key_consumption, 0);

                final_buffers.emplace(station, station_initial_buffer + keys_received - total_key_consumption);
            }

            return {solution.GetMetadata(), solution.Observations(), final_buffers};
        }

        inline std::size_t Index(const GroundStation &station) const { return station_indices_.at(station); }

        inline const GroundStation &Station(std::size_t station_index) const { return stations_.at(station_index); }

        inline const std::vector<GroundStation> &Stations() const { return stations_; }

        inline double TransferShare(const GroundStation &station) const { return problem_->TransferShare(station); }

        inline double InitialBuffer(const GroundStation &station) const { return problem_->InitialBuffer(station); }

    protected:
        virtual void AppendMetadata(Metadata &metadata) {}

        virtual void Build(const boost::optional<Solution> &initial_solution) = 0;

        virtual void ReportResults(util::SolverStatus solver_status) {
            LOG(INFO) << "Solver status: " << solver_status;
        }

        virtual std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period> > GetObservations() const = 0;

        std::size_t dummy_station_index_;

        ExtendedProblem const *problem_;

        GRBEnv mip_environment_;
        GRBModel mip_model_;

    private:
        std::vector<GroundStation> stations_;
        std::unordered_map<GroundStation, std::size_t> station_indices_;
    };
}


#endif //QUAKE_BASE_MIP_MODEL_H
