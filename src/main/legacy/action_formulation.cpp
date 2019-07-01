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

#include "action_formulation.h"

#include "cp_solution.h"
#include "no_improvement_time_limit.h"
#include "step_increment_operator.h"
#include "step_decrement_operator.h"

quake::ActionFormulation::ActionFormulation(operations_research::Solver *solver, quake::InferredModel const *model,
                                            int64 actions, int64 min_multiplier, int64 max_multiplier)
        : solver_{solver},
          model_{model},
          actions_{actions},
          min_multiplier_{min_multiplier},
          max_multiplier_{max_multiplier} {
    CHECK_GE(actions, model_->StationNoDummyCount());
    CHECK_GE(model_->StationNoDummyCount(), 1);
}

void quake::ActionFormulation::Build() {
    // --> define stations
    // first N-1 stations must be regular stations
    for (auto action_index = 0; action_index < model_->StationNoDummyCount(); ++action_index) {
        stations_.push_back(solver_->MakeIntVar(1, model_->StationNoDummyCount()));
    }

    if (actions_ > model_->StationNoDummyCount()) {
        // a dummy station may be selected for remaining stations
        for (auto action_index = model_->StationNoDummyCount(); action_index < actions_; ++action_index) {
            stations_.push_back(solver_->MakeIntVar(0, model_->StationNoDummyCount()));
        }
    }

    for (auto action_index = 0; action_index < actions_; ++action_index) {
        active_.push_back(solver_->MakeIsDifferentCstVar(stations_[action_index], 0));
    }

    if (actions_ > model_->StationNoDummyCount()) {
        // no regular station may appear after a dummy station
        for (auto action_index = model_->StationNoDummyCount(); action_index < actions_; ++action_index) {
            solver_->AddConstraint(
                    solver_->MakeGreaterOrEqual(
                            solver_->MakeProd(stations_[action_index - 1], model_->StationNoDummyCount()),
                            stations_[action_index]));
        }

        // no two regular stations next to each other are the same
        for (auto station_index = 1; station_index < model_->StationNoDummyCount(); ++station_index) {
            solver_->AddConstraint(
                    solver_->MakeNonEquality(stations_[station_index - 1], stations_[station_index]));
        }

        // no two regular stations next to each other are the same unless they are dummy
        for (auto action_index = model_->StationNoDummyCount(); action_index < actions_; ++action_index) {
            solver_->AddConstraint(
                    solver_->MakeLessOrEqual(
                            active_[action_index - 1],
                            solver_->MakeIsDifferentVar(stations_[action_index - 1], stations_[action_index])
                    ));
        }

        // at least 1 transfer is dedicated to one station
        std::vector<int64> station_indices(model_->StationCount(), 0);
        for (auto station_index = 1; station_index < model_->StationCount(); ++station_index) {
            station_indices[station_index] = station_index;
        }

        std::vector<int64> min_station_occ(model_->StationCount(), 1);
        min_station_occ[0] = 0;

        std::vector<int64> max_station_occ(static_cast<std::size_t >(model_->StationCount()),
                                           actions_ - model_->StationNoDummyCount());
        solver_->AddConstraint(
                solver_->MakeDistribute(stations_, station_indices, min_station_occ, max_station_occ));
    } else {
        solver_->AddConstraint(solver_->MakeAllDifferent(stations_));
    }

    // --> define start times
    for (auto station_index = 0; station_index < actions_; ++station_index) {
        start_times_.push_back(solver_->MakeIntVar(0, model_->TimeRange()));
    }

    // --> define reconfiguration time
    for (auto station_index = 0; station_index < actions_; ++station_index) {
        reconfiguration_time_.push_back(solver_->MakeIntVar(0, model_->TimeRange()));
    }

    // first start time is zero
    solver_->AddConstraint(solver_->MakeEquality(start_times_[0], 0));

    // first reconfiguration time is zero
    solver_->AddConstraint(solver_->MakeEquality(reconfiguration_time_[0], 0));

    // start times are increasing
    for (auto station_index = 1; station_index < actions_; ++station_index) {
        solver_->AddConstraint(
                solver_->MakeLessOrEqual(start_times_[station_index - 1], start_times_[station_index]));
    }

    // start times that belong to a regular station are offset by switch time
    for (auto station_index = 1; station_index < actions_; ++station_index) {
        solver_->AddConstraint(
                solver_->MakeEquality(active_[station_index - 1],
                                      solver_->MakeIsLessVar(
                                              solver_->MakeSum(start_times_[station_index - 1],
                                                               solver_->MakeProd(
                                                                       active_[station_index - 1],
                                                                       solver_->MakeElement(
                                                                               model_->ReconfigurationTime(),
                                                                               start_times_[station_index - 1]))),
                                              start_times_[station_index])));
    }

    // start times of a dummy station is at the end of time range
    for (auto station_index = model_->StationNoDummyCount(); station_index < actions_; ++station_index) {
        solver_->AddConstraint(solver_->MakeEquality(active_[station_index],
                                                     solver_->MakeIsDifferentCstVar(start_times_[station_index],
                                                                                    model_->TimeRange())));

    }

    // reconfiguration times of a dummy station is zero
    for (auto station_index = model_->StationNoDummyCount(); station_index < actions_; ++station_index) {
        solver_->AddConstraint(
                solver_->MakeLessOrEqual(solver_->MakeDifference(1, active_[station_index]),
                                         solver_->MakeIsEqualCstVar(reconfiguration_time_[station_index], 0)));

    }

    // --> start index and end index
    for (auto action_index = 0; action_index < actions_; ++action_index) {
        station_offset_.push_back(solver_->MakeProd(stations_[action_index], model_->StationIndexMultiplier()));
    }

    start_index_.push_back(solver_->MakeSum(station_offset_[0], start_times_[0])->Var());
    for (auto action_index = 1; action_index < actions_; ++action_index) {
        start_index_.push_back(
                solver_->MakeSum(station_offset_[action_index],
                                 solver_->MakeSum(start_times_[action_index],
                                                  solver_->MakeProd(
                                                          active_[action_index],
                                                          solver_->MakeElement(model_->ReconfigurationTime(),
                                                                               start_times_[action_index]))))->Var());
    }

    const auto last_action = actions_ - 1;
    for (auto action_index = 1; action_index < actions_; ++action_index) {
        end_index_.push_back(
                solver_->MakeSum(station_offset_[action_index - 1], start_times_[action_index])->Var());
    }
    end_index_.push_back(
            solver_->MakeSum(station_offset_[last_action], model_->TimeRange())->Var());

    // --> last start index is less than end index
    solver_->AddConstraint(solver_->MakeLessOrEqual(start_index_[last_action], end_index_[last_action]));

    // --> action transfer
    for (auto station_index = 0; station_index < actions_; ++station_index) {
        action_transfers_.push_back(
                solver_->MakeDifference(solver_->MakeElement(model_->KeyRateCumulative1d(),
                                                             end_index_[station_index]->Var()),
                                        solver_->MakeElement(model_->KeyRateCumulative1d(),
                                                             start_index_[station_index]->Var()))->Var());
    }

    // action transfer must be greater than zero for a regular station
    for (auto action_index = 0; action_index < actions_; ++action_index) {
        solver_->AddConstraint(
                solver_->MakeEquality(
                        active_[action_index],
                        solver_->MakeIsGreaterCstVar(action_transfers_[action_index], 0)
                ));
    }

    // -> total transfer
    total_transfer_ = solver_->MakeSum(action_transfers_)->Var();

    // --> station transfer
    // TODO: bug here -> should count to <
    for (auto station_index = 0; station_index <= model_->StationCount(); ++station_index) {
        std::vector<operations_research::IntVar *> partial_transfers;
        for (auto action_index = 0; action_index < actions_; ++action_index) {
            partial_transfers.push_back(
                    solver_->MakeProd(
                            solver_->MakeIsEqualCstVar(stations_[action_index], station_index),
                            action_transfers_[action_index])->Var());
        }
        station_transfers_.push_back(solver_->MakeSum(partial_transfers)->Var());
    }

    if (model_->TotalKeyRate().HasUpperBound()) {
        solver_->AddConstraint(solver_->MakeLessOrEqual(total_transfer_, model_->TotalKeyRate().UpperBound()));
    }

    if (model_->StationKeyRate().HasUpperBound()) {
        for (auto station_index = 1; station_index <= model_->StationNoDummyCount(); ++station_index) {
            solver_->AddConstraint(
                    solver_->MakeLessOrEqual(station_transfers_[station_index],
                                             model_->StationKeyRate().UpperBound()));
        }
    }

// FIXME: enforce lower bound
// this can effectively work only if a feasible initial solution has been provided
//        if (model_->TotalKeyRate().HasLowerBound()) {
//            solver_->AddConstraint(solver_->MakeGreaterOrEqual(total_transfer_, model_->TotalKeyRate().LowerBound()));
//        }
//        if (model_->StationKeyRate().HasLowerBound()) {
//            for (auto station_index = 1; station_index <= model_->StationNoDummyCount(); ++station_index) {
//                solver_->AddConstraint(
//                        solver_->MakeGreaterOrEqual(station_transfers_[station_index],
//                                                    model_->StationKeyRate().LowerBound()));
//            }
//        }

    operations_research::IntExpr *const max_transfer_rate = solver_->MakeMax(station_transfers_);
    for (auto station_index = 1; station_index <= model_->StationNoDummyCount(); ++station_index) {
        solver_->AddConstraint(
                solver_->MakeLessOrEqual(solver_->MakeProd(max_transfer_rate, max_multiplier_),
                                         solver_->MakeProd(station_transfers_[station_index],
                                                           min_multiplier_)));
    }

    active_actions_ = solver_->MakeSum(active_)->Var();

    total_transfer_obj_ = solver_->MakeMaximize(total_transfer_, 1);

    solution_collector_ = solver_->MakeLastSolutionCollector();
    solution_collector_->Add(start_times_);
    solution_collector_->Add(stations_);
    solution_collector_->Add(action_transfers_);
    solution_collector_->Add(total_transfer_);
}

quake::CpSolution quake::ActionFormulation::Solve(boost::optional<int> failure_frequency,
                                                  boost::optional<boost::posix_time::time_duration> time_limit,
                                                  bool print_solutions) {
    operations_research::DecisionBuilder *const main_phase = CreateDefaultDecisionBuilder();
    std::vector<operations_research::SearchMonitor *> search_monitors = CreateSearchMonitors(failure_frequency,
                                                                                             time_limit);

    VLOG(1) << "Started solving the " << solver_->model_name() << " model";
    if (print_solutions) {
        solver_->NewSearch(main_phase, search_monitors);
        ListSolutions();
    } else {
        solver_->Solve(main_phase, search_monitors);
    }
    VLOG(1) << "Completed solving the " << solver_->model_name() << " model";

    return GetSolution();
}

quake::CpSolution quake::ActionFormulation::SolveWithLocalSearch(operations_research::Assignment *assignment,
                                                                 boost::optional<int> failure_frequency,
                                                                 boost::optional<boost::posix_time::time_duration> time_limit,
                                                                 bool print_solutions) {
    operations_research::DecisionBuilder *const main_phase = CreateDefaultDecisionBuilder();

    std::vector<operations_research::SearchMonitor *> search_monitors
            = CreateSearchMonitors(failure_frequency, time_limit);

    std::vector<operations_research::LocalSearchOperator *> operators{
            solver_->MakeOperator(start_times_, operations_research::Solver::LocalSearchOperators::INCREMENT),
            solver_->MakeOperator(start_times_, operations_research::Solver::LocalSearchOperators::DECREMENT),
            solver_->RevAlloc(new StepIncrementOperator(start_times_)),
            solver_->RevAlloc(new StepDecrementOperator(start_times_))
    };

    static const auto RESTART_OPERATORS = true;
    operations_research::LocalSearchPhaseParameters *const ls_parameters
            = solver_->MakeLocalSearchPhaseParameters(solver_->ConcatenateOperators(operators, RESTART_OPERATORS),
                                                      solver_->MakeSolveOnce(main_phase),
                                                      nullptr,
                                                      {solver_->MakeVariableDomainFilter()});

    operations_research::DecisionBuilder *ls_search_phase;
    if (assignment) {
        ls_search_phase = solver_->MakeLocalSearchPhase(assignment, ls_parameters);
    } else {
        ls_search_phase = solver_->MakeLocalSearchPhase(start_times_,
                                                        solver_->MakeSolveOnce(main_phase),
                                                        ls_parameters);
    }

    if (print_solutions) {
        static const auto LOG_FREQUENCY = 1000000;
        operations_research::SearchMonitor *const search_log
                = solver_->MakeSearchLog(LOG_FREQUENCY, total_transfer_obj_);
        search_monitors.insert(std::begin(search_monitors), search_log);
    }

    VLOG(1) << "Started solving the " << solver_->model_name() << " model";
    if (print_solutions) {
        solver_->NewSearch(ls_search_phase, search_monitors);
        ListSolutions();
    } else {
        solver_->Solve(ls_search_phase, search_monitors);
    }
    VLOG(1) << "Completed solving the " << solver_->model_name() << " model";
    return GetSolution();
}

quake::CpSolution quake::ActionFormulation::GetSolution() const {
    static const auto SOLUTION_NUMBER = 0;

    std::vector<quake::CpSolution::Job> actions;
    for (auto station_index = 1; station_index < actions_; ++station_index) {
        int64 duration = solution_collector_->Value(SOLUTION_NUMBER, start_times_[station_index])
                         - solution_collector_->Value(SOLUTION_NUMBER, start_times_[station_index - 1]);
        CHECK_GE(duration, 0);
        actions.emplace_back(solution_collector_->Value(SOLUTION_NUMBER, start_times_[station_index - 1]),
                             duration,
                             solution_collector_->Value(SOLUTION_NUMBER, stations_[station_index - 1]),
                             solution_collector_->Value(SOLUTION_NUMBER, action_transfers_[station_index - 1]));
    }
    const auto last_action = actions_ - 1;
    actions.emplace_back(solution_collector_->Value(SOLUTION_NUMBER, start_times_[last_action]),
                         model_->TimeRange() - solution_collector_->Value(SOLUTION_NUMBER, start_times_[last_action]),
                         solution_collector_->Value(SOLUTION_NUMBER, stations_[last_action]),
                         solution_collector_->Value(SOLUTION_NUMBER, action_transfers_[last_action]));

    quake::CpSolution solution{std::move(actions)};
    DCHECK_EQ(solution.TotalKeyRate(), solution_collector_->Value(SOLUTION_NUMBER, total_transfer_));
    return solution;
}

operations_research::Assignment *quake::ActionFormulation::CreateAssignment(const quake::CpSolution &solution) const {
    operations_research::Assignment *assignment = solver_->MakeAssignment();

    assignment->Add(stations_);
    assignment->Add(start_times_);
    auto action_index = 0;
    for (; action_index < solution.Jobs().size(); ++action_index) {
        const auto &action = solution.Jobs()[action_index];
        assignment->SetValue(stations_[action_index], action.Station());
        assignment->SetValue(start_times_[action_index], action.Start());
    }

    for (; action_index < stations_.size(); ++action_index) {
        assignment->SetValue(stations_[action_index], model_->DummyStationIndex());
        assignment->SetValue(start_times_[action_index], model_->TimeRange());
    }

    return assignment;
}

void quake::ActionFormulation::ListSolutions() const {
    while (solver_->NextSolution()) {
        std::ostringstream output_msg;
        output_msg << "IpSolution " << solver_->solutions() << std::endl;
        output_msg << "Transfer stations: " << stations_[0]->Value();
        for (auto station_index = 1; station_index < actions_; ++station_index) {
            output_msg << ", ";
            output_msg << stations_[station_index]->Value();
        }
        output_msg << std::endl;
        output_msg << "Transfer key rates: " << station_transfers_[0]->Value();
        for (auto station_index = 1; station_index < model_->StationCount(); ++station_index) {
            output_msg << ", ";
            output_msg << station_transfers_[station_index]->Value();
        }
        output_msg << std::endl;
        output_msg << "Total: " << total_transfer_->Value();

        LOG(INFO) << output_msg.str();

#if DEBUG
        CHECK_EQ(station_transfers_[0]->Value(), 0);
        for (auto action_index = 0; action_index < actions_; action_index++) {
            CHECK_GE(action_transfers_[action_index]->Value(), 0);
            CHECK_LE(start_index_[action_index]->Value(), end_index_[action_index]->Value());
            CHECK_LE(end_index_[action_index]->Value() - start_index_[action_index]->Value(),
                     model_->TimeRange());
            CHECK_EQ(start_index_[action_index]->Value() / model_->StationIndexMultiplier(),
                     stations_[action_index]->Value());
            CHECK_EQ(end_index_[action_index]->Value() / model_->StationIndexMultiplier(),
                     stations_[action_index]->Value());
        }

        for (auto station_index = 0; station_index < model_->StationCount(); ++station_index) {
            CHECK_GE(station_transfers_[station_index]->Value(), 0);
        }
#endif
    }

    LOG(INFO) << "Statistics: solutions=" << solver_->solutions() << " failures=" << solver_->failures();
}

operations_research::DecisionBuilder *quake::ActionFormulation::CreateDefaultDecisionBuilder() const {
    operations_research::DecisionBuilder *const active_actions_phase = solver_->MakePhase(active_actions_,
                                                                                          operations_research::Solver::CHOOSE_FIRST_UNBOUND,
                                                                                          operations_research::Solver::ASSIGN_MIN_VALUE);

    operations_research::DecisionBuilder *const stations_phase = solver_->MakePhase(stations_,
                                                                                    operations_research::Solver::CHOOSE_MIN_SIZE,
                                                                                    operations_research::Solver::ASSIGN_RANDOM_VALUE);

    operations_research::DecisionBuilder *const start_times_phase = solver_->MakePhase(start_times_,
                                                                                       operations_research::Solver::CHOOSE_MIN_SIZE,
                                                                                       operations_research::Solver::ASSIGN_RANDOM_VALUE);

    //active_actions_phase
    return solver_->Compose(stations_phase, start_times_phase);
}

std::vector<operations_research::SearchMonitor *> quake::ActionFormulation::CreateSearchMonitors(
        boost::optional<int> failure_frequency,
        boost::optional<boost::posix_time::time_duration> time_limit) const {
    std::vector<operations_research::SearchMonitor *> search_monitors;

    search_monitors.push_back(total_transfer_obj_);
    search_monitors.push_back(solution_collector_);

    if (failure_frequency) {
        search_monitors.push_back(solver_->MakeLubyRestart(failure_frequency.get()));
    }

    if (time_limit) {
        search_monitors.push_back(solver_->MakeTimeLimit(time_limit.get().total_milliseconds()));
    } else {
        static const auto MAXIMIZE_OBJ_VALUE = true;
        static const auto NO_BETTER_SOLUTION_TIME_LIMIT = boost::posix_time::seconds(5);

        search_monitors.push_back(
                solver_->RevAlloc(
                        new NoImprovementTimeLimit(solver_,
                                                   total_transfer_,
                                                   NO_BETTER_SOLUTION_TIME_LIMIT,
                                                   MAXIMIZE_OBJ_VALUE)));
    }

    return std::move(search_monitors);
}
