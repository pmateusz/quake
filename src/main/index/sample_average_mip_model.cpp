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

#include <numeric>
#include <utility>

#include <boost/config.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <util/math.h>

#include "sample_average_mip_model.h"
#include "fixed_discretisation_scheme.h"

quake::SampleAverageMipModel::SampleAverageMipModel(quake::InferredModel const *model,
                                                    boost::posix_time::time_duration time_step,
                                                    std::vector<quake::Forecast> forecasts, double target_index)
        : BaseIntervalMipModel(model, std::move(time_step), std::move(forecasts)),
          target_index_{target_index},
          callback_{} {}

void quake::SampleAverageMipModel::Build(const boost::optional<Solution> &solution) {
    // set solver configuration:
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseIntervalMipModel::Build(solution);

    // variable: objective
    // constraint: lambda is bounded from above
    const auto max_traffic_index = GetTrafficIndexUpperBound();
    target_distance_var_ = mip_model_.addVar(0.0, max_traffic_index, 0.0, GRB_CONTINUOUS, "index_distance");

    // objective:
    GRBLinExpr objective = target_distance_var_;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mip_model_.setObjective(objective);

    callback_ = std::make_unique<SampleAverageCallback>(*this, target_index_, target_distance_var_);
    mip_model_.setCallback(callback_.get());
}

double quake::SampleAverageMipModel::GetTrafficIndex(const quake::Solution &solution) const {
    return BaseIntervalMipModel::GetTrafficIndex(solution, forecasts_.front());
}

quake::SampleAverageMipModel::SampleAverageCallback::SampleAverageCallback(quake::SampleAverageMipModel &model,
                                                                           double target_index,
                                                                           GRBVar target_distance_var)
        : BendersCallback{model, target_index},
          target_distance_var_{target_distance_var},
          scenario_distance_{},
          scenario_by_distance_{},
          dual_values_{},
          dual_scenarios_{} {
    const auto num_scenarios = model_.NumScenarios();
    scenario_distance_.reserve(num_scenarios);
    scenario_by_distance_.reserve(num_scenarios);
    dual_values_.reserve(num_scenarios);
    dual_scenarios_.reserve(num_scenarios);
}

static bool
descending_comparator(const std::pair<std::size_t, double> &left, const std::pair<std::size_t, double> &right) {
    return left.second > right.second;
}

void quake::SampleAverageMipModel::SampleAverageCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    try {
        const auto num_scenarios = model_.NumScenarios();
        for (auto station_index = model_.FirstRegularStation(); station_index < model_.NumStations(); ++station_index) {
            // consider a sub-problem for each station
            // evaluate distance to the target index
            scenario_distance_.clear();
            for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                scenario_distance_.emplace_back(DistanceToTarget(scenario_index, station_index));
            }
            const auto distance_sum
                    = std::accumulate(std::cbegin(scenario_distance_), std::cend(scenario_distance_), 0.0);

            if (distance_sum > 0.0) {
                // average distance is positive in expectation - feasibility cut required
                // number of keys to deliver

                GRBLinExpr keys_transferred_expr = 0.0;
                for (const auto &transfer_interval : model_.StationIntervals(station_index)) {
                    double interval_coefficient = 0.0;
                    for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                        interval_coefficient
                                += model_.KeysTransferred(scenario_index, transfer_interval);
                    }
                    keys_transferred_expr += interval_coefficient * transfer_interval.Var;
                }

                GRBLinExpr cumulative_target_distance
                        = num_scenarios * target_traffic_index_
                          - (1.0 / model_.TransferShare(station_index)) * keys_transferred_expr;
                addLazy(cumulative_target_distance <= 0.0);
            } else {
                scenario_by_distance_.clear();
                dual_values_.clear();
                dual_scenarios_.clear();

                // finding max dual
                // check if optimality cut is required
                const auto max_element_it
                        = std::max_element(std::cbegin(scenario_distance_), std::cend(scenario_distance_));
                if (*max_element_it <= 0.0) {
                    // maximum dual objective is negative
                    continue;
                }
                // for some scenarios distance to index is positive - check if optimality cut is required

                // sort scenarios in descending order according to distance
                CHECK(scenario_by_distance_.empty());
                for (std::size_t scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                    scenario_by_distance_.emplace_back(scenario_index, scenario_distance_.at(scenario_index));
                }
                std::sort(std::begin(scenario_by_distance_), std::end(scenario_by_distance_), descending_comparator);

                // computing dual objective for possible combinations of scenarios
                for (auto scenario_count = 1; scenario_count < num_scenarios; ++scenario_count) {
                    double selected_distance_sum = 0.0;
                    for (auto scenario_pos = 0; scenario_pos < scenario_count; ++scenario_pos) {
                        selected_distance_sum += scenario_by_distance_.at(scenario_pos).second;
                    }
                    dual_values_.push_back(selected_distance_sum / (num_scenarios - scenario_count));
                }

                // finding scenario combinations with maximum dual objective
                const auto max_dual_it = std::max_element(std::cbegin(dual_values_), std::cend(dual_values_));
                const auto final_dual_value = std::max(*max_dual_it, 0.0);
                if (final_dual_value <= 0.0) {
                    // maximum dual objective is negative
                    continue;
                }

                // for convenience obtain store indices of scenarios which yield the largest dual objective
                const auto num_dual_scenarios = std::distance(std::cbegin(dual_values_), max_dual_it) + 1;
                for (auto scenario_pos = 0; scenario_pos < num_dual_scenarios; ++scenario_pos) {
                    dual_scenarios_.push_back(scenario_by_distance_.at(scenario_pos).first);
                }

                // check if constraint is satisfied
                const auto &station_intervals = model_.StationIntervals(station_index);
                const auto num_station_intervals = station_intervals.size();

                const auto num_other_scenarios = num_scenarios - num_dual_scenarios;
                CHECK_GE(num_other_scenarios, 1);

                for (const auto index : dual_scenarios_) {
                    CHECK_LE(index, num_scenarios);
                    CHECK_GE(index, 0);
                }

                // computing interval coefficients for the extreme point
                double keys_delivered_cumulative = 0.0;
                for (auto interval_index = 0; interval_index < num_station_intervals; ++interval_index) {
                    const auto &interval = station_intervals.at(interval_index);
                    double interval_keys_delivered = 0.0;
                    for (const auto scenario_index : dual_scenarios_) {
                        interval_keys_delivered += model_.KeysTransferred(scenario_index, interval);
                    }
                    keys_delivered_cumulative += interval_keys_delivered * util::IsActive(getSolution(interval.Var));
                }

                const auto primal_value = (num_dual_scenarios * target_traffic_index_
                                           - keys_delivered_cumulative / model_.TransferShare(station_index))
                                          / num_other_scenarios;
                util::check_near(primal_value, final_dual_value);
                if (util::is_surely_gt(primal_value, getSolution(target_distance_var_))) {
                    VLOG(1) << "Adding optimality cut: " << primal_value
                            << " < index (current value: " << getSolution(target_distance_var_) << ")";
                    GRBLinExpr keys_delivered_cumulative_expr;
                    for (auto interval_index = 0; interval_index < num_station_intervals; ++interval_index) {
                        const auto &interval = station_intervals.at(interval_index);
                        double interval_keys_delivered = 0.0;
                        for (const auto scenario_index : dual_scenarios_) {
                            interval_keys_delivered += model_.KeysTransferred(scenario_index, interval);
                        }
                        keys_delivered_cumulative_expr += interval_keys_delivered * interval.Var;
                    }

                    GRBLinExpr primal_expr
                            = (num_dual_scenarios * target_traffic_index_
                               - keys_delivered_cumulative_expr / model_.TransferShare(station_index))
                              / num_other_scenarios;

                    addLazy(primal_expr <= target_distance_var_);
                }
            }
        }
    } catch (const GRBException &ex) {
        LOG(FATAL) << "Solver exception: " << ex.getMessage() << " code: " << ex.getErrorCode();
    } catch (...) {
        LOG(FATAL) << "Unhandled exception" << boost::current_exception_diagnostic_information();
    }
}