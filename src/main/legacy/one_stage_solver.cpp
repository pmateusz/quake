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

#include <glog/logging.h>

#include "objective_function.h"
#include "one_stage_solver.h"
#include "lns_path_stage_operator.h"

quake::OneStageSolver::OneStageSolver(const quake::InferredModel *model,
                                      int64 min_transfer_size,
                                      const quake::SolverArguments *args) : InventoryFormulation(model,
                                                                                                 min_transfer_size),
                                                                            args_{args},
                                                                            solver_{"one_stage_solver"},
                                                                            final_buffer_{} {}

boost::optional<quake::CpSolution> quake::OneStageSolver::Solve(const quake::CpSolution &initial_guess) {
    for (auto &stage : Stages()) {
        stage.Build(&solver_);
    }

    operations_research::Assignment *const assignment = BuildAssignment(initial_guess);
    CHECK(solver_.CheckAssignment(assignment));

    const auto local_search_solution_opt = Solve(initial_guess, assignment);
    if (local_search_solution_opt) {
        Model()->CheckConsistency(local_search_solution_opt.get());
    }

    return local_search_solution_opt;
}

boost::optional<quake::CpSolution> quake::OneStageSolver::Solve(const CpSolution &initial_solution,
                                                                operations_research::Assignment *assignment) {
    auto &stages = Stages();

    std::vector<operations_research::IntVar *> keys_transferred{solver_.MakeIntConst(0)}; // dummy ground station
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        std::vector<operations_research::IntVar *> station_stage_keys_transferred;

        for (auto &stage :stages) {
            station_stage_keys_transferred.emplace_back(stage.Transfer(station_index));
        }
        keys_transferred.push_back(solver_.MakeSum(station_stage_keys_transferred)->Var());
    }
    keys_transferred_ = keys_transferred;

    const auto days_length = stages.size();

    CHECK(final_buffer_.empty());
    final_buffer_ = std::vector<operations_research::IntVar *>{solver_.MakeIntConst(0)}; // dummy ground station
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        operations_research::IntVar *station_final_buffer
                = solver_.MakeSum(
                        solver_.MakeDifference(keys_transferred[station_index],
                                               solver_.MakeIntConst(
                                                       days_length * Model()->KeyConsumption(station_index))),
                        Model()->InitialBuffer(station_index))->Var();
        final_buffer_.push_back(station_final_buffer);
    }

    ObjectiveFunction objective_function = BuildObjective(initial_solution);

    std::vector<operations_research::IntVar *> start_times;
    for (auto &stage : stages) {
        std::copy(std::cbegin(stage.StartTime()), std::cend(stage.StartTime()), std::back_inserter(start_times));
    }

    std::vector<operations_research::IntVar *> all_next;
    std::vector<std::vector<operations_research::IntVar *>> next_by_stage;
    std::vector<operations_research::IntVar *> all_start;
    std::vector<std::vector<operations_research::IntVar *>> start_by_stage;

    for (auto &stage: stages) {
        next_by_stage.push_back(stage.Next());
        std::copy(std::cbegin(stage.Next()), std::cend(stage.Next()), std::back_inserter(all_next));

        start_by_stage.push_back(stage.StartTime());
        std::copy(std::cbegin(stage.StartTime()), std::cend(stage.StartTime()), std::back_inserter(all_start));
    }

    std::vector<operations_research::LocalSearchOperator *> operators{
            solver_.RevAlloc(new LNSPathStageOperator(next_by_stage, start_by_stage)),
            solver_.MakeOperator(start_times, operations_research::Solver::LocalSearchOperators::INCREMENT),
            solver_.MakeOperator(start_times, operations_research::Solver::LocalSearchOperators::DECREMENT)
    };

    operations_research::DecisionBuilder *cost_function_builder;
    if (objective_function.IsMinimize()) {
        cost_function_builder = solver_.MakePhase(objective_function.CostVariable,
                                                  operations_research::Solver::IntVarStrategy::CHOOSE_FIRST_UNBOUND,
                                                  operations_research::Solver::IntValueStrategy::ASSIGN_MIN_VALUE);
    } else {
        cost_function_builder = solver_.MakePhase(objective_function.CostVariable,
                                                  operations_research::Solver::IntVarStrategy::CHOOSE_FIRST_UNBOUND,
                                                  operations_research::Solver::IntValueStrategy::ASSIGN_MAX_VALUE);
    }

    operations_research::DecisionBuilder *inner_next_lns_decision_builder
            = solver_.MakePhase(all_next,
                                operations_research::Solver::IntVarStrategy::CHOOSE_MAX_REGRET_ON_MIN,
                                operations_research::Solver::IntValueStrategy::ASSIGN_RANDOM_VALUE);

    operations_research::DecisionBuilder *inner_start_lns_decision_builder
            = solver_.MakePhase(all_start,
                                operations_research::Solver::IntVarStrategy::CHOOSE_MIN_SIZE_LOWEST_MAX,
                                operations_research::Solver::IntValueStrategy::ASSIGN_RANDOM_VALUE);

    operations_research::DecisionBuilder *complementary_lns_decision_builder = solver_.MakeSolveOnce(
            solver_.Compose(inner_next_lns_decision_builder,
                            inner_start_lns_decision_builder,
                            cost_function_builder),
            solver_.MakeConstantRestart(5000),
            solver_.MakeTimeLimit(5000)
    );

    static const auto RESTART_OPERATORS = true;
    operations_research::LocalSearchPhaseParameters *const ls_parameters
            = solver_.MakeLocalSearchPhaseParameters(solver_.ConcatenateOperators(operators, RESTART_OPERATORS),
                                                     complementary_lns_decision_builder,
                                                     solver_.MakeTimeLimit(5000),
                                                     {solver_.MakeVariableDomainFilter()});

    operations_research::DecisionBuilder *ls_search_phase = solver_.MakeLocalSearchPhase(assignment, ls_parameters);

    operations_research::SolutionCollector *solution_collector
            = solver_.MakeBestValueSolutionCollector(objective_function.IsMaximize());
    solution_collector->AddObjective(objective_function.CostVariable);
    for (auto &stage: stages) {
        solution_collector->Add(stage.StartTime());
        solution_collector->Add(stage.Next());
        solution_collector->Add(stage.StartTransferTime());
        solution_collector->Add(stage.EndTransferTime());
        solution_collector->Add(stage.Transfer());
    }
    solution_collector->Add(final_buffer_);

    std::vector<operations_research::SearchMonitor *> search_monitors;
    search_monitors.push_back(objective_function.OptimizeVariable);
    search_monitors.push_back(solution_collector);

    if (args_->PrintSolutions) {
        search_monitors.push_back(solver_.MakeSearchLog(25000, objective_function.OptimizeVariable));
    }

    AppendSearchMonitors(&solver_, objective_function, *args_, search_monitors);

    // make sure the time limit is not too small
    const auto solver_status = solver_.Solve(ls_search_phase, search_monitors);
    if (!solver_status) {
        LOG(WARNING) << "Infeasible solution";
        return boost::none;
    }

    std::vector<CpSolution::Job> temp_jobs;
    for (auto &stage : stages) {
        static const auto SOLUTION_NUMBER = 0;
        const auto stage_jobs = stage.GetJobs(SOLUTION_NUMBER, solution_collector);
        std::copy(std::cbegin(stage_jobs), std::cend(stage_jobs), std::back_inserter(temp_jobs));
    }

    std::vector<int64> final_buffer{0};
    for (auto station_index = 1; station_index < StationSize(); ++station_index) {
        final_buffer.push_back(solution_collector->Value(0, final_buffer_.at(station_index)));
    }

    CpSolution solution(std::move(temp_jobs), std::move(final_buffer));

    if (args_->PrintSolutions) {
        const auto &initial_buffer = Model()->InitialBuffer();

        int64 total_initial_buffer = std::accumulate(std::cbegin(initial_buffer),
                                                     std::cend(initial_buffer),
                                                     static_cast<int64>(0));

        std::ostringstream output_msg;

        for (auto stage_index = 0; stage_index < Stages().size(); ++stage_index) {
            const auto &stage = stages[stage_index];
            const auto &transfer = stage.Transfer();
            output_msg << "Stage " << stage_index + 1 << std::endl;
            for (auto station_index = 1; station_index < transfer.size(); ++station_index) {
                output_msg << " - " << Model()->Station(station_index)
                           << ": " << solution_collector->Value(0, transfer[station_index])
                           << std::endl;
            }
        }


        std::vector<int64> final_buffer_value{0};
        for (auto station_index = 1; station_index < StationSize(); ++station_index) {
            const auto station_buffer = initial_buffer[station_index]
                                        - days_length * Model()->KeyConsumption(station_index);
            CHECK_GE(station_buffer, 0) << Model()->Station(station_index);
            final_buffer_value.push_back(station_buffer + solution.TransferredKeys(station_index));
        }
        int64 total_final_buffer_value = std::accumulate(std::cbegin(final_buffer_value),
                                                         std::cend(final_buffer_value),
                                                         static_cast<int64>(0));

        const auto initial_buffer_ratio = [&initial_buffer, &total_initial_buffer](const auto station_index)
                -> double {
            return static_cast<double>(initial_buffer[station_index]) / total_initial_buffer;
        };

        const auto final_buffer_ratio = [&final_buffer_value, &total_final_buffer_value](const auto station_index)
                -> double {
            return static_cast<double>(final_buffer_value[station_index]) / total_final_buffer_value;
        };

        output_msg << "Final: " << std::endl;
        output_msg << " - Buffer Ratio: ";
        output_msg << Model()->Station(1) << ": " << initial_buffer_ratio(1) << " -> " << final_buffer_ratio(1);
        for (std::size_t station_index = 2; station_index < StationSize(); ++station_index) {
            output_msg << "; " << Model()->Station(station_index)
                       << ": " << initial_buffer_ratio(station_index)
                       << " -> " << final_buffer_ratio(station_index);
        }
        output_msg << std::endl;

        const auto absolute_errors = Model()->GetAbsoluteError(solution);
        const auto relative_errors = Model()->GetRelativeError(solution);

        output_msg << " - Relative Errors: ";
        output_msg << relative_errors[0].first.name() << ' ' << relative_errors[0].second;
        for (auto station_index = 1; station_index < Model()->StationCount(); ++station_index) {
            output_msg << "; ";
            output_msg << relative_errors[station_index].first.name()
                       << ' ' << relative_errors[station_index].second;
        }
        output_msg << std::endl;

        output_msg << " - Absolute Errors: ";
        output_msg << absolute_errors[0].first.name() << ' ' << absolute_errors[0].second;
        for (auto station_index = 1; station_index < Model()->StationCount(); ++station_index) {
            output_msg << "; ";
            output_msg << absolute_errors[station_index].first.name()
                       << ' ' << absolute_errors[station_index].second;
        }
        output_msg << std::endl;

        output_msg << " - Final Number of Keys Transferred: ";
        output_msg << Model()->Station(0) << ' ' << final_buffer_value[0];
        for (auto station_index = 1; station_index < Model()->StationCount(); ++station_index) {
            output_msg << "; ";
            output_msg << Model()->Station(station_index) << ' ' << final_buffer_value[station_index];
        }

        LOG(INFO) << output_msg.str();
    }

    return boost::make_optional(solution);
}

operations_research::Assignment *const quake::OneStageSolver::BuildAssignment(const quake::CpSolution &solution) {
    operations_research::Assignment *const assignment = solver_.MakeAssignment();
    for (auto &stage : Stages()) {
        auto &start_time = stage.StartTime();
        auto &next = stage.Next();

        assignment->Add(start_time);
        assignment->Add(next);

        std::vector<CpSolution::Job> stage_jobs;
        for (const auto &job : solution.Jobs()) {
            if ((job.Start() < stage.BeginStageIndex())
                || (job.Start() == stage.BeginStageIndex() && job.Station() == Model()->DummyStationIndex())) {
                continue;
            }

            if ((job.Start() >= stage.EndStageIndex()) && (job.Station() != Model()->DummyStationIndex())) {
                break;
            }

            stage_jobs.push_back(job);
        }

        std::sort(std::begin(stage_jobs), std::end(stage_jobs), [](const CpSolution::Job &left,
                                                                   const CpSolution::Job &right) -> bool {
            return left.Start() <= right.Start();
        });

        const auto next_size = next.size();
        std::vector<int64> next_values(next_size, 0);
        for (auto next_index = 0; next_index < next_values.size(); ++next_index) {
            next_values[next_index] = next_index;
        }

        std::size_t job_index = 0;
        int64 node_index = 0;
        while (!stage.IsEndNode(node_index) && job_index < stage_jobs.size()) {
            next_values[node_index] = stage_jobs[job_index].Station();
            node_index = stage_jobs[job_index].Station();
            ++job_index;
        }
        if (!stage.IsEndNode(node_index)) {
            next_values[node_index] = stage.EndNode();
        }

        const auto job_size = stage_jobs.size();
        for (job_index = 0; job_index < job_size; ++job_index) {
            const auto &job = stage_jobs[job_index];
            assignment->SetValue(start_time[job.Station()], job.Start());
        }

        for (auto next_index = 0; next_index < next_size; ++next_index) {
            assignment->SetValue(next[next_index], next_values[next_index]);
        }

        assignment->SetValue(start_time[stage.StartNode()], stage.BeginStageIndex());
        assignment->SetValue(start_time[stage.EndNode()], stage.EndStageIndex());
    }
    return assignment;
}
