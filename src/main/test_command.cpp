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

#include "test_command.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <fstream>

#include <boost/math/constants/constants.hpp>

#include <glog/logging.h>

#include <ortools/constraint_solver/constraint_solveri.h>
#include <ortools/constraint_solver/constraint_solver.h>

#include "util/error.h"
#include "util/ortools.h"

#include "minizinc_reader.h"
#include "minizinc_data_model.h"

void quake::TestCommand::Run() {
    LowerBound();

    std::string file_path{"/home/pmateusz/dev/quake/data/minizinc/all_1sec_int.dzn"};
    std::ifstream input_stream;
    input_stream.open(file_path);
    if (input_stream.bad()) {
        throw util::OnFailedReadInput(file_path);
    }

    MiniZincReader<std::ifstream, int64> reader{std::move(input_stream)};
    auto raw_data_model = reader.Read();
    reader.Close();

    MiniZincDataModel<int64, int64> data_model(std::move(raw_data_model));

    std::vector<int64> key_rate_1d;
    for (const auto &station_key_rate : data_model.KeyRate()) {
        for (const auto &value : station_key_rate) {
            key_rate_1d.push_back(static_cast<int64>(round(value)));
        }
    }

    std::vector<int64> key_rate_cumul_1d;
    for (const auto &station_key_rate_cumul : data_model.KeyRate()) {
        for (const auto &value : station_key_rate_cumul) {
            key_rate_cumul_1d.push_back(static_cast<int64>(round(value)));
        }
    }

    const auto min_max_element = std::minmax_element(std::cbegin(key_rate_cumul_1d), std::cend(key_rate_cumul_1d));
    const auto min_value = *min_max_element.first;
    const auto max_value = *min_max_element.second;


    operations_research::Solver solver("quake");

    const auto stations = data_model.Stations().size();
    const auto stations_no_dummy = static_cast<int64>(data_model.Stations().size() - 1);
    const auto actions = stations_no_dummy;
    const auto time_range = data_model.Time().size();

    CHECK_LE(stations_no_dummy, actions);

    // upper bound
    std::vector<operations_research::IntVar *> station;
    std::vector<operations_research::IntVar *> transfer;

    for (auto time_index = 0; time_index < time_range; ++time_index) {
        station.push_back(solver.MakeIntVar(1, stations_no_dummy));
    }

    for (auto time_index = 0; time_index < time_range; ++time_index) {
        transfer.push_back(solver.MakeIntVar(min_value, max_value));
    }

    for (auto time_index = 0; time_index < time_range; ++time_index) {
        std::vector<int64> local_transfer_rates;
        for (auto station_index = 0; station_index < stations; ++station_index) {
            local_transfer_rates.push_back(key_rate_1d[station_index * time_range + time_index]);
        }

        transfer.push_back(solver.MakeElement(local_transfer_rates, station[time_index])->Var());
    }

    for (auto time_index = 0; time_index < time_range; ++time_index) {
        std::vector<int64> allowed_values;
        for (auto station_index = 1; station_index < stations; ++station_index) {
            allowed_values.push_back(key_rate_1d[station_index * time_range + time_index]);
        }

        solver.AddConstraint(solver.MakeMemberCt(transfer[time_index], allowed_values));
    }

    std::vector<operations_research::IntVar *> station_transfer;
    for (auto station_index = 1; station_index < stations; ++station_index) {
        std::vector<operations_research::IntVar *> cost_element;
        for (auto time_index = 0; time_index < time_range; ++time_index) {
            cost_element.push_back(solver.MakeProd(solver.MakeIsEqualCstVar(station[time_index], station_index),
                                                   transfer[time_index])->Var()
            );
        }
        station_transfer.push_back(solver.MakeSum(cost_element)->Var());
    }

    operations_research::IntExpr *const max_transfer_rate = solver.MakeMax(station_transfer);
    for (auto station_index = 1; station_index < stations; ++station_index) {
        solver.AddConstraint(solver.MakeLessOrEqual(solver.MakeProd(max_transfer_rate, 10),
                                                    solver.MakeProd(station_transfer[station_index - 1], 11)));
    }

    operations_research::IntExpr *const total_transfer = solver.MakeSum(transfer);

    operations_research::OptimizeVar *const objective = solver.MakeMaximize(total_transfer->Var(), 1);

    operations_research::DecisionBuilder *const transfer_phase = solver.MakePhase(transfer,
                                                                                  operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                                                                                  operations_research::Solver::ASSIGN_MAX_VALUE);

    operations_research::DecisionBuilder *const station_phase = solver.MakePhase(station,
                                                                                 operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                                                                                 operations_research::Solver::ASSIGN_RANDOM_VALUE);

    operations_research::DecisionBuilder *const main_phase = solver.Compose(station_phase, transfer_phase);

    LOG(INFO) << "Started search";

    solver.NewSearch(main_phase, objective);
    auto solutions = 0;
    while (solver.NextSolution()) {
        LOG(INFO) << "IpSolution: " << solutions;
        for (auto station_index = 1; station_index < stations; ++station_index) {
            LOG(INFO) << util::GetHumanFriendlyValue(station_transfer[station_index - 1]);
        }

        ++solutions;
    }
    solver.EndSearch();

    LOG(INFO) << "Solutions: " << solutions;
    LOG(INFO) << "Failures: " << solver.failures();
    LOG(INFO) << "Time: " << solver.wall_time();
}

void quake::TestCommand::TrigonometricExample() {
    operations_research::Solver solver("test");

    const auto column = 180;

    const auto start_min = 0;
    const auto start_max = column;
    const auto duration_min = 1;
    const auto duration_max = column;
    const auto end_min = 0;
    const auto end_max = column;
    const auto optional = false;

    std::vector<int64> values;
    for (auto index = 0; index < column; ++index) {
        values.push_back(
                static_cast<int64>(round(
                        1000.0 * sin(index * 2.0 * boost::math::constants::pi<double>() / 360.0))));
    }

    for (auto index = 0; index < column; ++index) {
        values.push_back(
                static_cast<int64>(round(
                        1000.0 * cos(index * 2.0 * boost::math::constants::pi<double>() / 360.0))));
    }

    std::vector<int64> cumulative_values{values};
    for (auto index = 1; index < column; ++index) {
        cumulative_values[index] = cumulative_values[index] + cumulative_values[index - 1];
    }

    for (auto index = column + 1; index < cumulative_values.size(); ++index) {
        cumulative_values[index] = cumulative_values[index] + cumulative_values[index - 1];
    }

    const auto min_max_element = std::minmax_element(std::cbegin(values), std::cend(values));
    const auto min_value = *min_max_element.first;
    const auto max_value = *min_max_element.second;

    operations_research::IntervalVar *const left = solver.MakeIntervalVar(start_min, start_max,
                                                                          duration_min, duration_max,
                                                                          end_min, end_max,
                                                                          optional, "left");

    operations_research::IntervalVar *const right = solver.MakeIntervalVar(start_min, start_max,
                                                                           duration_min, duration_max,
                                                                           end_min, end_max,
                                                                           optional, "right");

    operations_research::IntVar *const left_station = solver.MakeIntVar(0, 1, "left_fun");

    operations_research::IntVar *const right_station = solver.MakeIntVar(0, 1, "right_fun");

    solver.AddConstraint(
            solver.MakeGreaterOrEqual(left->DurationExpr(), solver.MakeIsEqualCstVar(right->DurationExpr(), 0)));

    operations_research::IntExpr *const left_end_index = solver.MakeSum(solver.MakeProd(left_station, column),
                                                                        left->EndExpr());

    operations_research::IntExpr *const left_start_index = solver.MakeSum(solver.MakeProd(left_station, column),
                                                                          left->StartExpr());

    operations_research::IntExpr *const left_exp = solver.MakeDifference(
            solver.MakeElement(cumulative_values,
                               solver.MakeSum(solver.MakeProd(left_station, column), left->EndExpr())->Var()),
            solver.MakeElement(cumulative_values,
                               solver.MakeSum(solver.MakeProd(left_station, column), left->StartExpr())->Var()));
    left_exp->set_name("left_value");

    operations_research::IntExpr *const right_exp = solver.MakeDifference(
            solver.MakeElement(cumulative_values,
                               solver.MakeSum(solver.MakeProd(right_station, column), right->EndExpr())->Var()),
            solver.MakeElement(cumulative_values,
                               solver.MakeSum(solver.MakeProd(right_station, column), right->StartExpr())->Var()));
    right_exp->set_name("right_value");

    operations_research::IntExpr *const total_value_exp = solver.MakeSum({left_exp->Var(), right_exp->Var()});

    operations_research::OptimizeVar *const objective = solver.MakeMaximize(total_value_exp->Var(), 1);

    solver.AddConstraint(solver.MakeLessOrEqual(left->StartExpr(), right->StartExpr()));

    solver.AddConstraint(solver.MakeDisjunctiveConstraint({left, right}, "disjunctive"));

    operations_research::DecisionBuilder *const start_phase
            = solver.MakePhase(
                    {left->StartExpr()->Var(), right->StartExpr()->Var()},
                    operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                    operations_research::Solver::ASSIGN_MIN_VALUE);

    operations_research::DecisionBuilder *const duration_phase
            = solver.MakePhase(
                    {left->DurationExpr()->Var(), right->DurationExpr()->Var()},
                    operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                    operations_research::Solver::ASSIGN_MAX_VALUE);

    operations_research::DecisionBuilder *const interval_phase
            = solver.MakePhase({left, right}, operations_research::Solver::INTERVAL_DEFAULT);

    operations_research::DecisionBuilder *const station_phase
            = solver.MakePhase(
                    {left_station->Var(), right_station->Var()},
                    operations_research::Solver::INT_VAR_DEFAULT,
                    operations_research::Solver::INT_VALUE_DEFAULT);

    operations_research::DecisionBuilder *const value_phase
            = solver.MakePhase(
                    total_value_exp->Var(),
                    operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                    operations_research::Solver::ASSIGN_MAX_VALUE);

    operations_research::DecisionBuilder *const main_decision_builder = solver.Compose(value_phase,
                                                                                       station_phase,
                                                                                       interval_phase);

    operations_research::SolutionCollector *const collector = solver.MakeLastSolutionCollector();
    collector->Add(left);
    collector->Add(right);

    solver.NewSearch(main_decision_builder, objective);
    auto solutions = 0;
    while (solver.NextSolution()) {
        LOG(INFO) << "IpSolution " << solutions;
        if (left->MustBePerformed()) {
            LOG(INFO) << left->name()
                      << " start: " << util::GetHumanFriendlyValue(left->StartExpr())
                      << " end: " << util::GetHumanFriendlyValue(left->EndExpr());
        } else {
            LOG(INFO) << left->name() << " - not performed";
        }

        if (right->MustBePerformed()) {
            LOG(INFO) << right->name()
                      << " start: " << util::GetHumanFriendlyValue(right->StartExpr())
                      << " end: " << util::GetHumanFriendlyValue(right->EndExpr());
        } else {
            LOG(INFO) << right->name() << " - not performed";
        }

        LOG(INFO) << util::GetHumanFriendlyValue(total_value_exp);

        ++solutions;
    }
    solver.EndSearch();

    LOG(INFO) << "Solutions: " << solutions;
    LOG(INFO) << "Failures: " << solver.failures();
    LOG(INFO) << "Time: " << solver.wall_time();
}

void quake::TestCommand::LowerBound() {
    std::string file_path{"/home/pmateusz/dev/quake/data/minizinc/all_1sec_int.dzn"};
    std::ifstream input_stream;
    input_stream.open(file_path);
    if (input_stream.bad()) {
        throw util::OnFailedReadInput(file_path);
    }

    MiniZincReader<std::ifstream, int64> reader{std::move(input_stream)};
    auto raw_data_model = reader.Read();
    reader.Close();

    MiniZincDataModel<int64, int64> data_model(std::move(raw_data_model));

    std::vector<int64> key_rate_cumulative_1d;
    for (const auto &station_key_rate_cumulative : data_model.KeyRateCumulative()) {
        for (const auto &value : station_key_rate_cumulative) {
            key_rate_cumulative_1d.push_back(static_cast<int64>(round(value)));
        }
    }

    const auto min_max_element = std::minmax_element(std::cbegin(key_rate_cumulative_1d),
                                                     std::cend(key_rate_cumulative_1d));
    const auto min_value = *min_max_element.first;
    const auto max_value = *min_max_element.second;


    operations_research::Solver solver("quake");

    const auto stations_no_dummy = static_cast<int64>(data_model.Stations().size() - 1);
    const auto actions = stations_no_dummy;
    const auto time_range = data_model.Time().size();

    CHECK_LE(stations_no_dummy, actions);

    // lower bound
    std::vector<operations_research::IntVar *> start_time;
    std::vector<operations_research::IntVar *> station;

    for (auto action_index = 0; action_index < actions; ++action_index) {
        station.push_back(solver.MakeIntVar(1, actions));
    }

    for (auto action_index = 0; action_index < actions; ++action_index) {
        start_time.push_back(solver.MakeIntVar(0, time_range));
    }

    solver.AddConstraint(solver.MakeEquality(start_time[0], 0));
    for (auto action_index = 1; action_index < actions; ++action_index) {
        solver.AddConstraint(
                solver.MakeLessOrEqual(solver.MakeSum(start_time[action_index - 1], data_model.SwitchDuration()),
                                       start_time[action_index]));
    }
    solver.AddConstraint(solver.MakeLessOrEqual(start_time[actions - 1], time_range - data_model.SwitchDuration()));

    std::vector<operations_research::IntVar *> duration;
    for (auto action_index = 1; action_index < actions; ++action_index) {
        duration.push_back(solver.MakeDifference(start_time[action_index], start_time[action_index - 1])->Var());
    }
    duration.push_back(solver.MakeDifference(time_range, start_time[actions - 1])->Var());

    std::vector<operations_research::IntExpr *> station_offset;
    for (auto action_index = 0; action_index < actions; ++action_index) {
        station_offset.push_back(solver.MakeProd(station[action_index], time_range));
    }

    std::vector<operations_research::IntVar *> start_index;
    for (auto action_index = 0; action_index < actions; ++action_index) {
        start_index.push_back(solver.MakeSum(station_offset[action_index],
                                             solver.MakeSum(start_time[action_index],
                                                            data_model.SwitchDuration()))->Var());
    }

    std::vector<operations_research::IntVar *> end_index;
    for (auto action_index = 1; action_index < actions; ++action_index) {
        end_index.push_back(solver.MakeSum(station_offset[action_index - 1], start_time[action_index])->Var());
    }
    end_index.push_back(solver.MakeSum(station_offset[actions - 1], time_range)->Var());

    std::vector<operations_research::IntVar *> transfer_rate;
    for (auto action_index = 0; action_index < actions; ++action_index) {
        transfer_rate.push_back(
                solver.MakeDifference(solver.MakeElement(key_rate_cumulative_1d,
                                                         end_index[action_index]->Var()),
                                      solver.MakeElement(key_rate_cumulative_1d,
                                                         start_index[action_index]->Var()))->Var());
    }
    duration.push_back(solver.MakeDifference(time_range, start_time[actions - 1])->Var());

    solver.AddConstraint(solver.MakeAllDifferent(station));

    operations_research::IntVar *const total_transfer_rate = solver.MakeSum(transfer_rate)->Var();

    operations_research::IntExpr *const max_transfer_rate = solver.MakeMax(transfer_rate);
    for (auto action_index = 0; action_index < actions; ++action_index) {
        solver.AddConstraint(solver.MakeLessOrEqual(solver.MakeProd(max_transfer_rate, 10),
                                                    solver.MakeProd(transfer_rate[action_index], 11)));
    }

    operations_research::OptimizeVar *const objective = solver.MakeMaximize(total_transfer_rate, 1);

    operations_research::DecisionBuilder *const max_phase = solver.MakePhase(total_transfer_rate->Var(),
                                                                             operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                                                                             operations_research::Solver::ASSIGN_MAX_VALUE);

    operations_research::DecisionBuilder *const station_phase = solver.MakePhase(station,
                                                                                 operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                                                                                 operations_research::Solver::ASSIGN_RANDOM_VALUE);

    operations_research::DecisionBuilder *const start_time_phase = solver.MakePhase(start_time,
                                                                                    operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                                                                                    operations_research::Solver::ASSIGN_RANDOM_VALUE);

    operations_research::DecisionBuilder *const main_phase = solver.Compose(station_phase, start_time_phase);

    LOG(INFO) << "Started search";

    solver.NewSearch(main_phase, objective);
    auto solutions = 0;
    while (solver.NextSolution()) {
        LOG(INFO) << "IpSolution: " << solutions;
        for (auto action_index = 0; action_index < actions; ++action_index) {
            LOG(INFO) << "start: " << util::GetHumanFriendlyValue(start_time[action_index])
                      << " station: " << util::GetHumanFriendlyValue(station[action_index])
                      << " transfer: " << util::GetHumanFriendlyValue(transfer_rate[action_index]);
        }

        ++solutions;
    }
    solver.EndSearch();

    LOG(INFO) << "Solutions: " << solutions;
    LOG(INFO) << "Failures: " << solver.failures();
    LOG(INFO) << "Time: " << solver.wall_time();
}
