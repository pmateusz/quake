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

            mip_model_.set(GRB_IntParam_Presolve, GRB_PRESOLVE_AGGRESSIVE);
            mip_model_.set(GRB_IntParam_MIPFocus, GRB_MIPFOCUS_BALANCED);
            mip_model_.set(GRB_IntParam_Cuts, GRB_CUTS_AUTO);

            try {
                Build(initial_guess);
            } catch (const GRBException &exception) {
                LOG(FATAL) << "Solver exception " << exception.getMessage()
                           << " error code: " << exception.getErrorCode();
            }

            mip_model_.optimize();

            using namespace util;

            const auto solver_status_code = mip_model_.get(GRB_IntAttr_Status);
            const auto solver_status = static_cast<SolverStatus>(solver_status_code);
            CHECK_NE(solver_status, SolverStatus::Loaded);
            CHECK_NE(solver_status, SolverStatus::InProgress);
            CHECK_NE(solver_status, SolverStatus::Numeric);
            CHECK_NE(solver_status, SolverStatus::Unbounded);
            CHECK_NE(solver_status, SolverStatus::InfiniteOrUnbounded);
            CHECK_NE(solver_status, SolverStatus::Infeasible);
            ReportResults(solver_status);

            auto observations = GetObservations();

            std::unordered_map<quake::GroundStation, int64> final_buffers;
            for (const auto &station: Stations()) {
                if (station == GroundStation::None) {
                    final_buffers.emplace(station, 0);
                    continue;
                }

                // number of keys transferred
                const auto find_observations_it = observations.find(station);
                double keys_received = 0.0;
                if (find_observations_it != std::end(observations)) {
                    for (const auto &observation_window : find_observations_it->second) {
                        keys_received += problem_->KeyRate(station, observation_window, ExtendedProblem::WeatherSample::Forecast);
                    }
                }

                // keys consumed
                const auto total_days = problem_->ObservationPeriod().length().total_seconds() / boost::posix_time::hours(24).total_seconds();
                const auto total_key_consumption = problem_->KeyConsumption(station) * total_days;

                // initial buffer
                const auto station_initial_buffer = problem_->InitialBuffer(station);

                CHECK_GE(station_initial_buffer - total_key_consumption, 0);

                final_buffers.emplace(station, station_initial_buffer + keys_received - total_key_consumption);
            }

            Solution solution{std::move(observations), std::move(final_buffers)};

            Validator validator{*problem_};
            validator.Validate(solution);

            return boost::make_optional(solution);
        }

        inline std::size_t Index(const GroundStation &station) const { return station_indices_.at(station); }

        inline const GroundStation &Station(std::size_t station_index) const { return stations_.at(station_index); }

        inline const std::vector<GroundStation> &Stations() const { return stations_; }

        inline double TransferShare(const GroundStation &station) const { return problem_->TransferShare(station); }

        inline double InitialBuffer(const GroundStation &station) const { return problem_->InitialBuffer(station); }

    protected:

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
