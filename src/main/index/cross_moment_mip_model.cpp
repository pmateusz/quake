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

#include "cross_moment_mip_model.h"

#include <boost/config.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>

#include "util/math.h"

quake::CrossMomentMipModel::CrossMomentMipModel(quake::InferredModel const *model,
                                                boost::posix_time::time_duration time_step,
                                                std::vector<quake::Forecast> forecasts, double target_index)
        : BaseIntervalMipModel(model, std::move(time_step), std::move(forecasts)),
          target_index_{target_index} {}

void quake::CrossMomentMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // variable: traffic index
    const auto max_traffic_index = GetTrafficIndexUpperBound();
    std::vector<GRBVar> station_target_distance;
    for (auto station_index = 0; station_index < num_stations_; ++station_index) {
        std::stringstream label;
        label << "s" << station_index << "_distance_index";
        station_target_distance.push_back(mip_model_.addVar(0.0, max_traffic_index, 0.0, GRB_CONTINUOUS, label.str()));
    }

    // constraints: quadratic constraints
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        const auto scenario_matrix = ExtractScenarioMatrix(station_index);
        const auto mean_vector = util::mean(scenario_matrix);
        const auto covariance_matrix = util::covariance(scenario_matrix);
        const auto num_intervals = intervals_.at(station_index).size();
        const auto &station_intervals = intervals_.at(station_index);

        LOG(INFO) << station_index;
        LOG(INFO) << mean_vector;
        LOG(INFO) << covariance_matrix;

        double initial_covariance_multiplication = 0.0;
        double initial_mean_distance = 0.0;

        GRBQuadExpr covariance_multiplication = 0.0;
        GRBLinExpr mean_distance = 0.0;
        for (auto left_interval_index = 0; left_interval_index < num_intervals; ++left_interval_index) {
            const auto &left_interval = station_intervals.at(left_interval_index);
            for (auto right_interval_index = 0; right_interval_index < num_intervals; ++right_interval_index) {
                const auto &right_interval = station_intervals.at(left_interval_index);
                covariance_multiplication += left_interval.Var * right_interval.Var *
                                             covariance_matrix(right_interval_index, left_interval_index);

                if (IsSetInSolution(left_interval, *solution) && IsSetInSolution(right_interval, *solution)) {
                    initial_covariance_multiplication += covariance_matrix(right_interval_index, left_interval_index);
                }
            }

            mean_distance += mean_vector(left_interval_index) * left_interval.Var;
            if (IsSetInSolution(left_interval, *solution)) {
                initial_mean_distance += mean_vector(left_interval_index);
            }
        }
        const auto expected_keys_to_deliver = ExpectedKeysDelivered(station_index);
        GRBQuadExpr right_expr =
                4 * station_target_distance.at(station_index) * (mean_distance - expected_keys_to_deliver);
        mip_model_.addQConstr(covariance_multiplication <= right_expr, "covariance_bound");
//        LOG(INFO) << "Station " << station_index << ": " << initial_covariance_multiplication
//                  << " <= 4 * lambda * " << initial_mean_distance - expected_keys_to_deliver;
    }

    // constraint: satisfaction on average
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        const auto &station_intervals = intervals_.at(station_index);
        const auto num_intervals = station_intervals.size();
        GRBLinExpr keys_transferred = 0.0;
        for (auto interval_index = 0; interval_index < num_intervals; ++interval_index) {
            const auto &interval = station_intervals.at(interval_index);
            keys_transferred += interval.Var * scenario_pool_.MeanKeysTransferred(interval);
        }
        const auto expected_keys_to_deliver = ExpectedKeysDelivered(station_index);

        std::stringstream constraint_label;
        constraint_label << "transfer_more_than_expected_station_" << station_index;
        mip_model_.addConstr(expected_keys_to_deliver <= keys_transferred, constraint_label.str());
    }

    // objective:
    GRBLinExpr objective = 0;
    for (auto station_index = 0; station_index < num_stations_; ++station_index) {
        objective += station_target_distance.at(station_index);
    }

    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mip_model_.setObjective(objective);
}


double quake::CrossMomentMipModel::GetTrafficIndex(const quake::Solution &solution) const {
    return BaseIntervalMipModel::GetTrafficIndex(solution, forecasts_.front());
}

boost::numeric::ublas::matrix<double>
quake::CrossMomentMipModel::ExtractScenarioMatrix(std::size_t station_index) const {
    const auto num_intervals = intervals_.at(station_index).size();
    const auto num_scenarios = scenario_pool_.size();

    boost::numeric::ublas::matrix<double> scenario_matrix(num_scenarios, num_intervals);
    for (auto interval_index = 0; interval_index < num_intervals; ++interval_index) {
        const auto &interval = intervals_.at(station_index).at(interval_index);
        for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
            scenario_matrix(scenario_index, interval_index) = scenario_pool_.KeysTransferred(scenario_index, interval);
        }
    }
    return scenario_matrix;
}

double quake::CrossMomentMipModel::ExpectedKeysDelivered(std::size_t station_index) const {
    return target_index_ * model_->TransferShare(station_index);
}

bool quake::CrossMomentMipModel::IsSetInSolution(const quake::BaseInterval &interval,
                                                 const quake::Solution &solution) const {
    const auto station = model_->Station(interval.StationIndex);
    for (const auto &window : solution.ObservationWindows(station)) {
        const auto begin_index = model_->GetStartIndex(window.begin());
        const auto end_index = model_->GetEndIndex(window.end());
        if (end_index <= interval.Begin) { continue; }
        if (interval.End < begin_index) { break; }
        if (begin_index <= interval.Begin && interval.End <= end_index) { return true; }
    }
    return false;
}
