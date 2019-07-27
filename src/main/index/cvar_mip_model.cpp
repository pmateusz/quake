//
// Copyright 2019 Mateusz Polnik
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

#include <utility>

#include <boost/config.hpp>
#include <boost/exception/diagnostic_information.hpp>

#include "util/math.h"
#include "cvar_mip_model.h"

quake::CVarMipModel::CVarMipModel(ExtendedProblem const *problem,
                                  boost::posix_time::time_duration interval_step,
                                  std::vector<Forecast> forecasts,
                                  double target_index,
                                  double epsilon)
        : BaseIntervalMipModel(problem, std::move(interval_step), std::move(forecasts)),
          target_index_{target_index},
          epsilon_{epsilon} {}

void quake::CVarMipModel::Build(const boost::optional<quake::Solution> &solution) {
    // set solver configuration:
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseIntervalMipModel::Build(solution);

    const auto &forecast = Forecasts().front();

    // variable: traffic index
    const auto traffic_index_ub = BaseIntervalMipModel::GetTrafficIndexUpperBound();
    auto traffic_index = mip_model_.addVar(0, traffic_index_ub, 0, GRB_CONTINUOUS, "traffic_index");

    // constraint: bound the traffic index from above
    for (const auto &station : Stations()) {
        if (station == GroundStation::None) { continue; }

        GRBLinExpr keys_transferred = 0;
        for (const auto &interval : StationIntervals(station)) {
            keys_transferred += problem_->KeyRate(station, interval.Period(), forecast) * interval.Var();
        }

        mip_model_.addConstr(TransferShare(station) * traffic_index <= keys_transferred);
    }

    // objective
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    GRBLinExpr objective = traffic_index;
    mip_model_.setObjective(objective);

    // callback
    callback_ = std::make_unique<CVarCallback>(*this, target_index_, epsilon_);
    mip_model_.setCallback(callback_.get());
}

void quake::CVarMipModel::AppendMetadata(quake::Metadata &metadata) {
    BaseIntervalMipModel::AppendMetadata(metadata);

    metadata.SetProperty(Metadata::Property::SolutionMethod, Metadata::SolutionMethod::ConditionalValueAtRisk);
    metadata.SetProperty(Metadata::Property::ScenariosNumber, NumScenarios());
    metadata.SetProperty(Metadata::Property::TargetTrafficIndex, target_index_);
    metadata.SetProperty(Metadata::Property::Epsilon, epsilon_);
}

quake::CVarMipModel::CVarCallback::CVarCallback(quake::CVarMipModel &model, double target_index, double epsilon)
        : BendersCallback{model, target_index},
          epsilon_{epsilon} {}

static bool descending_comparator(const std::pair<std::size_t, double> &left, const std::pair<std::size_t, double> &right) {
    return left.second > right.second;
}

void quake::CVarMipModel::CVarCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    try {
        const auto num_scenarios = model_.NumScenarios();
        for (const auto &station : model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            std::vector<double> scenario_distance(num_scenarios, 0.0);
            for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                scenario_distance.at(scenario_index) = DistanceToTarget(scenario_index, station);
            }
            const auto distance_sum
                    = std::accumulate(std::cbegin(scenario_distance), std::cend(scenario_distance), 0.0);
            if (distance_sum > 0.0) {
                // average distance is positive in expectation - feasibility cut required
                // number of keys to deliver
                double keys_transferred = 0.0;
                GRBLinExpr keys_transferred_expr = 0.0;
                for (const auto &transfer_interval : model_.StationIntervals(station)) {
                    double interval_coefficient = 0.0;
                    for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                        interval_coefficient
                                += model_.KeyRate(scenario_index, station, transfer_interval.Period());
                    }
                    keys_transferred_expr += interval_coefficient * transfer_interval.Var();
                    keys_transferred += interval_coefficient * util::IsActive(getSolution(transfer_interval.Var()));
                }

                GRBLinExpr cumulative_target_distance_expr = num_scenarios * target_traffic_index_
                                                             - keys_transferred_expr / model_.TransferShare(station);

                double final_cumulative_target_distance = num_scenarios * target_traffic_index_
                                                          - keys_transferred / model_.TransferShare(station);

                util::check_near(distance_sum, final_cumulative_target_distance);
                VLOG(1) << "Adding feasibility cut: " << distance_sum
                        << " (cumulative target distance) <= 0 (at station: " << station << ")";
                addLazy(cumulative_target_distance_expr <= 0.0);
            } else {
                // sort scenarios descending by distance
                std::vector<std::pair<std::size_t, double> > scenario_by_distance;
                scenario_by_distance.reserve(num_scenarios);
                for (std::size_t scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
                    scenario_by_distance.emplace_back(scenario_index, scenario_distance.at(scenario_index));
                }
                std::sort(std::begin(scenario_by_distance), std::end(scenario_by_distance), descending_comparator);

                // compute dual value
                const auto denominator = epsilon_ * num_scenarios;
                const auto scenarios_to_sum = static_cast<std::size_t>(std::floor(epsilon_ * num_scenarios));

                CHECK_GT(scenarios_to_sum, 0);
                CHECK_LE(scenarios_to_sum, num_scenarios);

                double delay_cumulative_scenarios = 0.0;
                for (auto scenario_pos = 0; scenario_pos < scenarios_to_sum; ++scenario_pos) {
                    delay_cumulative_scenarios += scenario_by_distance.at(scenario_pos).second;
                }

                double delay_last_scenario = 0.0;
                if (scenarios_to_sum < num_scenarios) {
                    delay_last_scenario = scenario_by_distance.at(scenarios_to_sum).second;
                }
                const auto final_dual_value = delay_cumulative_scenarios / denominator
                                              + (1.0 - static_cast<double>(scenarios_to_sum) / denominator) *
                                                delay_last_scenario;
                if (final_dual_value > 0.0) {
                    GRBLinExpr cumulative_keys_scenarios_expr = 0;
                    double cumulative_keys_scenarios = 0;
                    for (const auto &interval : model_.StationIntervals(station)) {
                        double interval_cumulative_key_transferred = 0;
                        for (auto scenario_pos = 0; scenario_pos < scenarios_to_sum; ++scenario_pos) {
                            const auto scenario_index = scenario_by_distance.at(scenario_pos).first;
                            interval_cumulative_key_transferred += model_.KeyRate(scenario_index, station, interval.Period());
                        }

                        cumulative_keys_scenarios_expr
                                += interval_cumulative_key_transferred * interval.Var();
                        cumulative_keys_scenarios
                                += interval_cumulative_key_transferred * util::IsActive(getSolution(interval.Var()));
                    }
                    GRBLinExpr cumulative_distance_scenarios_expr = scenarios_to_sum * target_traffic_index_
                                                                    - cumulative_keys_scenarios_expr /
                                                                      model_.TransferShare(station);
                    double cumulative_distance_scenarios = scenarios_to_sum * target_traffic_index_ -
                                                           cumulative_keys_scenarios /
                                                           model_.TransferShare(station);


                    double distance_last_scenario = 0;
                    GRBLinExpr distance_last_scenario_expr = 0;
                    if (scenarios_to_sum < num_scenarios) {
                        double keys_last_scenario = 0;
                        GRBLinExpr keys_last_scenario_expr = 0;

                        const auto scenario_index = scenario_by_distance.at(scenarios_to_sum).first;
                        for (const auto &interval : model_.StationIntervals(station)) {
                            keys_last_scenario += model_.KeyRate(scenario_index, station, interval.Period())
                                                  * util::IsActive(getSolution(interval.Var()));
                            keys_last_scenario_expr += model_.KeyRate(scenario_index, station, interval.Period()) * interval.Var();
                        }

                        distance_last_scenario =
                                target_traffic_index_ - keys_last_scenario / model_.TransferShare(station);
                        distance_last_scenario_expr =
                                target_traffic_index_ - keys_last_scenario_expr / model_.TransferShare(station);
                    }

                    const auto primal_value = cumulative_distance_scenarios / denominator
                                              + (1.0 - scenarios_to_sum / denominator) * distance_last_scenario;
                    util::check_near(primal_value, final_dual_value);
                    VLOG(1) << "Adding optimality cut: " << final_dual_value << " <= 0 (at station: " << station << ")";
                    addLazy(cumulative_distance_scenarios_expr / denominator
                            + (1.0 - scenarios_to_sum / denominator) * distance_last_scenario_expr <= 0);
                }
            }
        }

    } catch (const GRBException &ex) {
        LOG(FATAL) << "Solver exception: " << ex.getMessage() << " code: " << ex.getErrorCode();
    } catch (...) {
        LOG(FATAL) << "Unhandled exception" << boost::current_exception_diagnostic_information();
    }
}
