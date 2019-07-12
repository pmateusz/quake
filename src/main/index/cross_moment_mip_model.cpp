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

quake::CrossMomentMipModel::CrossMomentMipModel(ExtendedProblem const *problem,
                                                boost::posix_time::time_duration interval_step,
                                                std::vector<quake::Forecast> forecasts, double target_index)
        : BaseIntervalMipModel(problem, std::move(interval_step), std::move(forecasts)),
          target_index_{target_index} {}

void quake::CrossMomentMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // variable: traffic index
    const auto max_traffic_index = GetTrafficIndexUpperBound();
    std::vector<GRBVar> station_target_distance;
    for (const auto &station : Stations()) {
        std::stringstream label;
        label << "s" << Index(station) << "_distance_index";
        station_target_distance.push_back(mip_model_.addVar(0.0, max_traffic_index, 0.0, GRB_CONTINUOUS, label.str()));
    }

    // constraints: quadratic constraints
    for (const auto &station : Stations()) {
        if (station == GroundStation::None) { continue; }

        const auto station_index = Index(station);
        const auto scenario_matrix = ExtractScenarioMatrix(station);
        const auto mean_vector = util::mean(scenario_matrix);
        const auto covariance_matrix = util::covariance(scenario_matrix);
        const auto &station_intervals = StationIntervals(station);
        const auto num_intervals = station_intervals.size();

        double initial_covariance_multiplication = 0.0;
        double initial_mean_distance = 0.0;

        GRBQuadExpr covariance_multiplication = 0.0;
        GRBLinExpr mean_distance = 0.0;
        for (auto left_interval_index = 0; left_interval_index < num_intervals; ++left_interval_index) {
            const auto &left_interval = station_intervals.at(left_interval_index);
            for (auto right_interval_index = 0; right_interval_index < num_intervals; ++right_interval_index) {
                const auto &right_interval = station_intervals.at(left_interval_index);
                covariance_multiplication += left_interval.Var() * right_interval.Var() *
                                             covariance_matrix(right_interval_index, left_interval_index);

                if (IsSetInSolution(left_interval, *solution) && IsSetInSolution(right_interval, *solution)) {
                    initial_covariance_multiplication += covariance_matrix(right_interval_index, left_interval_index);
                }
            }

            mean_distance += mean_vector(left_interval_index) * left_interval.Var();
            if (IsSetInSolution(left_interval, *solution)) {
                initial_mean_distance += mean_vector(left_interval_index);
            }
        }
        const auto expected_keys_to_deliver = ExpectedKeysDelivered(station);
        GRBQuadExpr right_expr =
                4 * station_target_distance.at(station_index) * (mean_distance - expected_keys_to_deliver);
        mip_model_.addQConstr(covariance_multiplication <= right_expr, "covariance_bound");
//        LOG(INFO) << "Station " << station_index << ": " << initial_covariance_multiplication
//                  << " <= 4 * lambda * " << initial_mean_distance - expected_keys_to_deliver;
    }

    // constraint: satisfaction on average
    for (const auto &station : Stations()) {
        if (station == GroundStation::None) { continue; }

        GRBLinExpr keys_transferred = 0.0;
        for (const auto &interval : StationIntervals(station)) {
            keys_transferred += interval.Var() * KeyRate(station, interval.Period());
        }
        const auto expected_keys_to_deliver = ExpectedKeysDelivered(station);

        std::stringstream constraint_label;
        constraint_label << "transfer_more_than_expected_station_" << Index(station);
        mip_model_.addConstr(expected_keys_to_deliver <= keys_transferred, constraint_label.str());
    }

    // objective:
    GRBLinExpr objective = 0;
    for (auto &local_target_distance : station_target_distance) {
        objective += local_target_distance;
    }

    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mip_model_.setObjective(objective);
}


double quake::CrossMomentMipModel::GetTrafficIndex(const Solution &solution) const {
    return BaseIntervalMipModel::GetTrafficIndex(solution, Forecasts().front());
}

boost::numeric::ublas::matrix<double>
quake::CrossMomentMipModel::ExtractScenarioMatrix(const GroundStation &station) const {
    const auto &station_intervals = StationIntervals(station);
    const auto num_intervals = station_intervals.size();
    const auto num_scenarios = NumScenarios();

    boost::numeric::ublas::matrix<double> scenario_matrix(num_scenarios, num_intervals);
    for (auto interval_index = 0; interval_index < num_intervals; ++interval_index) {
        const auto &interval = station_intervals.at(interval_index);
        for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
            scenario_matrix(scenario_index, interval_index) = KeyRate(scenario_index, station, interval.Period());
        }
    }
    return scenario_matrix;
}

double quake::CrossMomentMipModel::ExpectedKeysDelivered(const GroundStation &station) const {
    return target_index_ * TransferShare(station);
}

bool quake::CrossMomentMipModel::IsSetInSolution(const robust::IntervalVar &interval, const Solution &solution) const {
    const auto station = Station(interval.StationIndex());
    for (const auto &window : solution.ObservationWindows(station)) {
        if (window.is_after(interval.Period().end())) { break; }
        if (window.contains(interval.Period())) { return true; }
    }
    return false;
}
