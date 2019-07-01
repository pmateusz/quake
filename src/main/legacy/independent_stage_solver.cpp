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

#include <thread>
#include <memory>

#include <boost/config.hpp>
#include <boost/exception_ptr.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/optional/optional_io.hpp>

#include <glog/logging.h>

#include "independent_stage_solver.h"
#include "solver_arguments.h"
#include "objective_function.h"


class ObjectiveFunctionBuilder {
public:
    explicit ObjectiveFunctionBuilder(operations_research::Solver *solver)
            : solver_{solver} {}

    virtual void AppendComponent(operations_research::IntVar *const total,
                                 operations_research::IntVar *const fraction,
                                 double expected_ratio) {
        if (abs(expected_ratio) < 0.001) {
            AppendComponentScaled(total, fraction, static_cast<int64>(0));
        } else {
            const auto scaled_component = static_cast<int64>(expected_ratio * SCALING_FACTOR);
            CHECK_GT(scaled_component, 0);

            AppendComponentScaled(total, fraction, scaled_component);
        }
    }

    virtual quake::ObjectiveFunction Build() = 0;

protected:
    virtual void AppendComponentScaled(operations_research::IntVar *total,
                                       operations_research::IntVar *fraction,
                                       int64 expected_ratio_scaled) = 0;

    static const auto SCALING_FACTOR = 1000;

    operations_research::Solver *solver_;
};

class MinMaxRelativeErrorObjectiveBuilder : public ObjectiveFunctionBuilder {
public:
    explicit MinMaxRelativeErrorObjectiveBuilder(operations_research::Solver *solver)
            : ObjectiveFunctionBuilder(solver) {}

    virtual quake::ObjectiveFunction Build() {
        operations_research::IntVar *const cost = solver_->MakeMax(components_)->Var();
        operations_research::OptimizeVar *const objective_function = solver_->MakeMinimize(cost, 1);
        return {true, cost, objective_function};
    }

protected:
    virtual void AppendComponentScaled(operations_research::IntVar *total,
                                       operations_research::IntVar *fraction,
                                       int64 expected_ratio_scaled) {
        operations_research::IntVar *const expected_fraction = solver_->MakeProd(total,
                                                                                 expected_ratio_scaled)->Var();
        // the scaling factor in the nominator cannot be too high, otherwise solver will report a failure
        operations_research::IntVar *const component = solver_->MakeDiv(
                solver_->MakeProd(solver_->MakeAbs(
                        solver_->MakeDifference(expected_fraction,
                                                solver_->MakeProd(fraction, SCALING_FACTOR))), 100),
                expected_fraction)->Var();
        components_.push_back(component);
    }

private:
    std::vector<operations_research::IntVar *> components_;
};

class CumulativeAbsoluteErrorObjectiveBuilder : public ObjectiveFunctionBuilder {
public:
    CumulativeAbsoluteErrorObjectiveBuilder(operations_research::Solver *solver)
            : ObjectiveFunctionBuilder(solver) {}

    quake::ObjectiveFunction Build() {
        operations_research::IntVar *const cost = solver_->MakeSum(components_)->Var();
        operations_research::OptimizeVar *const objective_function = solver_->MakeMinimize(cost, 1);
        return {true, cost, objective_function};
    }

protected:
    virtual void AppendComponentScaled(operations_research::IntVar *total,
                                       operations_research::IntVar *fraction,
                                       int64 expected_ratio_scaled) override {
        operations_research::IntVar *const component = solver_->MakeSquare(
                solver_->MakeDifference(
                        solver_->MakeProd(total, expected_ratio_scaled),
                        solver_->MakeProd(fraction, SCALING_FACTOR)))->Var();
        components_.push_back(component);
    }

private:
    std::vector<operations_research::IntVar *> components_;
};

std::unique_ptr<ObjectiveFunctionBuilder> CreateObjectiveFunctionBuilder(quake::ObjectiveType function_type,
                                                                         operations_research::Solver *solver) {
    switch (function_type) {
        case quake::ObjectiveType::Default:
        case quake::ObjectiveType::MinTotalAbsoluteError:
            return std::make_unique<CumulativeAbsoluteErrorObjectiveBuilder>(solver);
        case quake::ObjectiveType::MinMaxRelativeError:
            return std::make_unique<MinMaxRelativeErrorObjectiveBuilder>(solver);
        case quake::ObjectiveType::Envelope:
        case quake::ObjectiveType::SurplusEnvelope:
            LOG(WARNING) << "Objective function type '"
                         << function_type
                         << "' is not supported for problem decomposition. Falling back to '"
                         << quake::ObjectiveType::MinTotalAbsoluteError << "'";
            return std::make_unique<CumulativeAbsoluteErrorObjectiveBuilder>(solver);
        default:
            LOG(FATAL) << "Unknown function type: " << function_type;
    }
}

template<typename KeyType, typename ValueType>
static bool DescendingComparer(const std::pair<KeyType, ValueType> &left, const std::pair<KeyType, ValueType> &right) {
    return std::get<1>(right) < std::get<1>(left);
};

template<typename ErrorFormula>
std::vector<std::pair<quake::GroundStation, double> > GetError(const quake::CpSolution &solution,
                                                               const quake::InferredModel &model) {
    static const ErrorFormula formula;

    std::vector<std::pair<quake::GroundStation, double> > errors;
    const double total_key_rate = solution.TotalKeyRate();
    CHECK_GT(total_key_rate, 0.0);
    for (std::size_t station_index = 0; station_index < model.StationCount(); ++station_index) {
        const double station_key_rate = solution.TransferredKeys(station_index);
        const double station_share = model.TransferShare(station_index);

        double error = 0.0;
        if (station_share > 0.0 && total_key_rate > 0.0) {
            const double expected_station_key_rate = station_share * total_key_rate;

            CHECK_GT(expected_station_key_rate, 0.0);
            error = formula(station_key_rate, expected_station_key_rate);
        }
        errors.emplace_back(model.Station(station_index), error);
    }

    std::sort(std::begin(errors),
              std::end(errors),
              DescendingComparer<quake::GroundStation, double>);

    return errors;
}

struct AbsoluteError {
    double operator()(double actual_value, double expected_value) const {
        return abs(actual_value - expected_value);
    }
};

struct RelativeError {
    double operator()(double actual_value, double expected_value) const {
        return abs((actual_value - expected_value) / expected_value);
    }
};

std::vector<std::pair<quake::GroundStation, double> > GetAbsoluteError(const quake::CpSolution &solution,
                                                                       const quake::InferredModel &model) {
    return GetError<AbsoluteError>(solution, model);
}

std::vector<std::pair<quake::GroundStation, double> > GetRelativeError(const quake::CpSolution &solution,
                                                                       const quake::InferredModel &model) {
    return GetError<RelativeError>(solution, model);
}

class StageSolutionPrinter {
public:
    StageSolutionPrinter(operations_research::Solver const *solver,
                         quake::StagePathFormulation const *stage,
                         quake::InferredModel const *model)
            : solver_{solver},
              stage_{stage},
              model_{model} {}

    std::string operator()() const {
        const auto &next = stage_->Next();
        const auto &transfer = stage_->Transfer();
        const auto &start_time = stage_->StartTime();

        std::vector<int64> next_values;
        for (auto &next_var : next) {
            next_values.push_back(next_var->Value());
        }

        std::vector<quake::CpSolution::Job> path_nodes;
        std::vector<quake::CpSolution::Job> missing_nodes;

        int64 current_node_index = next_values[0];
        while (!stage_->IsEndNode(current_node_index)) {
            const auto local_station = current_node_index;
            const auto local_start_time = start_time[current_node_index]->Value();
            const auto local_transfer = transfer[current_node_index]->Value();
            const auto local_next_node = next_values[current_node_index];
            const auto local_duration = start_time[local_next_node]->Value() - local_start_time;
            path_nodes.emplace_back(local_start_time, local_duration, local_station, local_transfer);

            current_node_index = local_next_node;
        }

        std::unordered_set<int64> path_node_set;
        for (const auto &path_node : path_nodes) {
            path_node_set.insert(path_node.Station());
        }

        for (auto local_station = 1; local_station < model_->StationCount(); ++local_station) {
            if (path_node_set.find(local_station) == std::cend(path_node_set)) {
                const auto local_start_time = start_time[local_station]->Value();
                const auto local_transfer = transfer[local_station]->Value();
                const auto local_duration = stage_->EndStageIndex() - start_time[local_station]->Value();
                missing_nodes.emplace_back(local_start_time, local_duration, local_station, local_transfer);
            }
        }

        std::ostringstream output_msg;

        const auto append_job = [&output_msg, this](const quake::CpSolution::Job &job) -> void {
            static const auto SEPARATOR = "; ";

            output_msg << this->model_->Station(job.Station())
                       << SEPARATOR << job.Start()
                       << SEPARATOR << job.Duration()
                       << SEPARATOR << job.KeysTransferred();
        };

        if (path_nodes.empty()) {
            output_msg << " - Served: []" << std::endl;
        } else {
            output_msg << " - Served: ";
            append_job(path_nodes[0]);
            for (auto node_index = 1; node_index < path_nodes.size(); ++node_index) {
                output_msg << " -> ";
                append_job(path_nodes[node_index]);
            }
            output_msg << std::endl;
        }

        if (missing_nodes.empty()) {
            output_msg << " - Missed: []" << std::endl;
        } else {
            output_msg << " - Missed:";
            for (const auto &job : missing_nodes) {
                output_msg << ' ';
                append_job(job);
            }
            output_msg << std::endl;
        }

        std::vector<quake::CpSolution::Job> solution_nodes{path_nodes};
        std::copy(std::cbegin(missing_nodes), std::cend(missing_nodes), std::back_inserter(solution_nodes));
        quake::CpSolution local_solution{std::move(solution_nodes)};

        const auto absolute_errors = GetAbsoluteError(local_solution, *model_);
        const auto relative_errors = GetRelativeError(local_solution, *model_);

        output_msg << " - Relative Errors: ";
        output_msg << relative_errors[0].first.name() << ' ' << relative_errors[0].second;
        for (auto station_index = 1; station_index < model_->StationCount(); ++station_index) {
            output_msg << "; ";
            output_msg << relative_errors[station_index].first.name()
                       << ' ' << relative_errors[station_index].second;
        }
        output_msg << std::endl;

        output_msg << " - Absolute Errors: ";
        output_msg << absolute_errors[0].first.name() << ' ' << absolute_errors[0].second;
        for (auto station_index = 1; station_index < model_->StationCount(); ++station_index) {
            output_msg << "; ";
            output_msg << absolute_errors[station_index].first.name()
                       << ' ' << absolute_errors[station_index].second;
        }

        return output_msg.str();
    }

private:
    operations_research::Solver const *solver_;
    quake::StagePathFormulation const *stage_;
    quake::InferredModel const *model_;
};

quake::IndependentStageSolver::IndependentStageSolver(quake::InferredModel const *model)
        : InventoryFormulation(model) {}

quake::CpSolution quake::IndependentStageSolver::Solve(const quake::SolverArguments &args) {
    auto &stages = Stages();

    std::vector<std::vector<CpSolution::Job> > stage_results(stages.size(),
                                                             std::vector<CpSolution::Job>{});
    std::atomic<int> stage_index{0};
    std::vector<std::thread> workers;
    const auto stage_worker_body = [&stage_index, &stage_results, &args, &stages, this]() -> void {
        try {
            while (true) {
                auto local_stage_index = stage_index++;
                if (local_stage_index >= Stages().size()) {
                    return;
                }
                std::stringstream solver_name;
                solver_name << "Stage-" << local_stage_index;

                auto jobs = SolveIndependentStage(solver_name.str(), stages[local_stage_index], args);
                if (jobs.empty()) {
                    stages[local_stage_index].Clear();
                    stages[local_stage_index].SetMinTransferSize(0);
                    jobs = SolveIndependentStage(solver_name.str(), stages[local_stage_index], args);
                }

                CHECK(!jobs.empty()) << "Failed to solve stage" << local_stage_index;
                stage_results[local_stage_index] = jobs;
            }
        }
        catch (...) {
            LOG(FATAL) << "Thread terminated due to unhandled exception: "
                       << boost::current_exception_diagnostic_information();
        }
    };

    const auto workers_count = std::min(4u, std::thread::hardware_concurrency());
    for (auto thread_index = 0; thread_index < workers_count; ++thread_index) {
        workers.emplace_back(stage_worker_body);
    }

    for (auto &worker : workers) {
        worker.join();
    }

    std::vector<int64> total_keys_transferred(StationSize(), 0);
    std::vector<CpSolution::Job> aggregated_jobs;
    for (const auto &stage_result : stage_results) {
        for (const auto &job : stage_result) {
            total_keys_transferred[job.Station()] += job.KeysTransferred();
        }

        std::copy(std::cbegin(stage_result), std::cend(stage_result), std::back_inserter(aggregated_jobs));
    }

    const auto days_length = stages.size();
    std::vector<int64> final_buffer{0};
    for (auto station_index = 1; station_index < StationSize(); ++station_index) {
        const auto station_final_buffer =
                Model()->InitialBuffer(station_index) - days_length * Model()->KeyConsumption(station_index) +
                total_keys_transferred[station_index];
        CHECK_GE(station_final_buffer, 0);
        final_buffer.push_back(station_final_buffer);
    }

    const auto solution = CpSolution{std::move(aggregated_jobs), std::move(final_buffer)};
    for (auto station_index = 1; station_index < StationSize(); ++station_index) {
        CHECK_EQ(solution.TransferredKeys(station_index), total_keys_transferred[station_index]);
    }

    Model()->CheckConsistency(solution);

    return solution;
}

std::vector<quake::CpSolution::Job>
quake::IndependentStageSolver::SolveIndependentStage(const std::string &problem_name,
                                                     quake::StagePathFormulation &stage,
                                                     const quake::SolverArguments &args) {
    operations_research::Solver solver(problem_name);
    stage.Build(&solver);

    auto &next = stage.Next();
    auto &start_time = stage.StartTime();

    std::unique_ptr<ObjectiveFunctionBuilder> objective_builder
            = CreateObjectiveFunctionBuilder(args.ProblemObjectiveFunction, &solver);
    operations_research::IntVar *const total_keys = solver.MakeSum(stage.Transfer())->Var();
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        objective_builder->AppendComponent(total_keys,
                                           stage.Transfer(station_index),
                                           Model()->TransferShare(station_index));
    }
    const auto objective_function = objective_builder->Build();

    std::vector<operations_research::LocalSearchOperator *> inner_local_search_operators{
            solver.MakeOperator(stage.StartTime(), operations_research::Solver::LocalSearchOperators::INCREMENT),
            solver.MakeOperator(stage.StartTime(), operations_research::Solver::LocalSearchOperators::DECREMENT),
            solver.MakeOperator(stage.StartTime(), operations_research::Solver::LocalSearchOperators::SIMPLELNS)
    };

    static const auto MAX_LOCAL_SEARCH_TIME = 6 * 1000;
    static const auto MAX_LOCAL_SEARCH_FAILURES = 12 * 1024;

    operations_research::LocalSearchPhaseParameters *const inner_local_search_parameters
            = solver.MakeLocalSearchPhaseParameters(
                    solver.ConcatenateOperators(inner_local_search_operators, true),
                    nullptr,
                    solver.MakeLimit(MAX_LOCAL_SEARCH_TIME, kint64max, MAX_LOCAL_SEARCH_FAILURES, kint64max, true),
                    {solver.MakeVariableDomainFilter()});

    operations_research::DecisionBuilder *inner_decision_builder = solver.MakeLocalSearchPhase(
            start_time,
            solver.MakePhase(stage.StartTime(),
                             operations_research::Solver::IntVarStrategy::CHOOSE_MIN_SIZE_LOWEST_MAX,
                             operations_research::Solver::IntValueStrategy::ASSIGN_RANDOM_VALUE),
            inner_local_search_parameters);

    operations_research::DecisionBuilder *decision_builder
            = solver.MakeLocalSearchPhase(next,
                                          solver.MakePhase(next,
                                                           operations_research::Solver::IntVarStrategy::CHOOSE_MAX_REGRET_ON_MIN,
                                                           operations_research::Solver::IntValueStrategy::ASSIGN_RANDOM_VALUE),
                                          solver.MakeLocalSearchPhaseParameters(
                                                  solver.ConcatenateOperators(
                                                          {
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::EXCHANGE),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::TWOOPT),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::RELOCATE),
//                                                                  solver.MakeOperator(next,
//                                                                                      operations_research::Solver::MAKEACTIVE),
//                                                                  solver.MakeOperator(next,
//                                                                                      operations_research::Solver::MAKEINACTIVE),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::PATHLNS)
                                                          }),
                                                  inner_decision_builder,
                                                  nullptr,
                                                  {solver.MakeVariableDomainFilter()}));

    operations_research::SolutionCollector *solution_collector
            = solver.MakeBestValueSolutionCollector(objective_function.IsMaximize());
    solution_collector->AddObjective(objective_function.CostVariable);
    solution_collector->Add(next);
    solution_collector->Add(start_time);
    solution_collector->Add(stage.Transfer());
    solution_collector->Add(stage.StartTransferTime());
    solution_collector->Add(stage.EndTransferTime());

    std::vector<operations_research::SearchMonitor *> local_search_monitors;
    local_search_monitors.push_back(objective_function.OptimizeVariable);

    if (args.PrintSolutions) {
        StageSolutionPrinter solution_printer{&solver, &stage, Model()};
        local_search_monitors.push_back(
                solver.MakeSearchLog(100, objective_function.OptimizeVariable, solution_printer));
    }

    local_search_monitors.push_back(solution_collector);

    AppendSearchMonitors(&solver, objective_function, args, local_search_monitors);

    const auto solution_status = solver.Solve(decision_builder, local_search_monitors);
    LOG(INFO) << "Statistics: solutions=" << solver.solutions() << " failures=" << solver.failures();
    VLOG(1) << "Completed solving the " << solver.model_name() << " model";

    if (solution_status) {
#if DEBUG
        stage.CheckConsistency(0, solution_collector);
#endif
        const auto result = stage.GetJobs(0, solution_collector);
        stage.Clear();

        return result;
    }

    return {};
}
