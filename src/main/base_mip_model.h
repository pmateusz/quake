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

#include "inferred_model.h"
#include "base_interval.h"

namespace quake {

    class BaseMipModel {
    public:
        BaseMipModel(quake::InferredModel const *model, boost::posix_time::time_duration time_step)
                : model_{model},
                  time_step_{std::move(time_step)},
                  num_time_{static_cast<std::size_t>(model_->TimeRange())},
                  num_stations_{static_cast<std::size_t>(model_->StationCount())},
                  dummy_station_{0},
                  first_regular_station_{1},
                  mip_environment_{},
                  mip_model_{mip_environment_} {}

        boost::optional<quake::Solution> Solve(const boost::optional<boost::posix_time::time_duration> &time_limit_opt,
                                               const boost::optional<double> &gap_opt,
                                               const boost::optional<Solution> &solution) {
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
                Build(solution);
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
            return boost::make_optional(model_->Create(std::move(observations)));
        }
        
        inline std::size_t FirstRegularStation() const { return first_regular_station_; }

        inline std::size_t NumStations() const { return num_stations_; }

        inline double TransferShare(std::size_t station_index) const { return model_->TransferShare(station_index); }

    protected:
        virtual void Build(const boost::optional<Solution> &initial_solution) = 0;

        virtual void ReportResults(util::SolverStatus solver_status) {
            LOG(INFO) << "Solver status: " << solver_status;
        }

        virtual std::unordered_map<GroundStation,
                std::vector<boost::posix_time::time_period> > GetObservations() const = 0;

        quake::InferredModel const *model_;
        boost::posix_time::time_duration time_step_;

        std::size_t num_time_;
        std::size_t num_stations_;
        std::size_t dummy_station_;
        std::size_t first_regular_station_;

        GRBEnv mip_environment_;
        GRBModel mip_model_;
    };
}


#endif //QUAKE_BASE_MIP_MODEL_H
