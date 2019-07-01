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

#include "block_intervals_mip_model.h"

quake::BlockIntervalsMipModel::BlockIntervalsMipModel(quake::InferredModel const *model,
                                                      Forecast forecast,
                                                      boost::posix_time::time_duration time_step)
        : BaseIntervalMipModel(model, std::move(time_step), std::vector<Forecast>{std::move(forecast)}) {}

double quake::BlockIntervalsMipModel::GetTrafficIndex(const quake::Solution &solution) const {
    return BaseIntervalMipModel::GetTrafficIndex(solution, forecasts_.front());
}

void quake::BlockIntervalsMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // obtain maximum lambdas
    const auto last_time_slot = model_->TimeRange();
    std::vector<double> max_station_keys{0};
    std::vector<double> max_station_lambda{0.0};
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        const auto station_callback = model_->GetWeatherAdjustedCumulativeKeyRate(station_index);
        const auto max_keys = station_callback(last_time_slot);
        max_station_keys.push_back(max_keys);

        const auto best_station_lambda = ceil(
                static_cast<double>(max_keys) / model_->TransferShare(station_index));
        max_station_lambda.push_back(best_station_lambda);
    }

    // constraint: lambda is bounded from above
    const double max_lambda = *std::max_element(std::cbegin(max_station_lambda), std::cend(max_station_lambda));
    const double max_keys_sum
            = std::accumulate(std::cbegin(max_station_keys), std::cend(max_station_keys), 0.0);
    lambda_ = mip_model_.addVar(0.0, max_lambda, 0.0, GRB_CONTINUOUS, "lambda");
    std::vector<GRBLinExpr> keys_delivered(num_stations_);
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        for (const auto &interval : intervals_[station_index]) {
            CHECK_EQ(interval.StationIndex, interval.StationIndex);
            keys_delivered.at(interval.StationIndex) += scenario_pool_.KeysTransferred(0, interval) * interval.Var;
        }
    }

    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        mip_model_.addConstr(
                model_->TransferShare(station_index) * lambda_
                <= model_->InitialBuffer(station_index) + keys_delivered.at(station_index));
    }

    // objective function
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);

    GRBLinExpr first_objective = lambda_;
    mip_model_.setObjectiveN(first_objective, 0, 10);

    GRBLinExpr total_keys = 0.0;
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        for (const auto &interval : intervals_[station_index]) {
            total_keys += scenario_pool_.KeysTransferred(0, interval) * interval.Var;
        }
    }

    total_keys_ = mip_model_.addVar(0, max_keys_sum, 0.0, GRB_CONTINUOUS, "total_keys");
    mip_model_.addConstr(total_keys_ == total_keys);
    GRBLinExpr second_objective = total_keys_;
    mip_model_.setObjectiveN(second_objective, 1, 5);

    auto first_obj_env = mip_model_.getMultiobjEnv(0);
    first_obj_env.set(GRB_DoubleParam_MIPGap, first_objective_mip_gap_);

    auto second_obj_env = mip_model_.getMultiobjEnv(1);
    second_obj_env.set(GRB_DoubleParam_MIPGap, second_objective_mip_gap_);
}

void quake::BlockIntervalsMipModel::ReportResults(quake::util::SolverStatus solver_status) {
    BaseMipModel::ReportResults(solver_status);

    std::stringstream msg;

    msg << "First objective stop MIP Gap: " << first_objective_mip_gap_ << std::endl;
    msg << "Second objective stop MIP Gap: " << second_objective_mip_gap_ << std::endl;

    msg << "Best solution:" << std::endl
        << " - lambda: " << lambda_.get(GRB_DoubleAttr_X) << std::endl
        << " - total keys: " << total_keys_.get(GRB_DoubleAttr_X) << std::endl;

    // Print number of solutions stored
    auto num_solutions = mip_model_.get(GRB_IntAttr_SolCount);
    msg << "Number of solutions found " << num_solutions << ":" << std::endl;
    msg << " solutions:" << std::endl;
    for (auto solution_index = 0; solution_index < num_solutions; ++solution_index) {
        mip_model_.set(GRB_IntParam_SolutionNumber, solution_index);

        mip_model_.set(GRB_IntParam_ObjNumber, 0);
        const auto lambda = mip_model_.get(GRB_DoubleAttr_ObjNVal);

        mip_model_.set(GRB_IntParam_ObjNumber, 1);
        const auto total_keys = mip_model_.get(GRB_DoubleAttr_ObjNVal);
        msg << " - " << lambda << ", " << total_keys << std::endl;
    }

    LOG(INFO) << msg.str();
}
