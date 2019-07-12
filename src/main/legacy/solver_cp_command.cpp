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

#include <algorithm>
#include <cmath>
#include <fstream>

#include <boost/format.hpp>
#include <boost/date_time.hpp>

#include <glog/logging.h>

#include <ortools/constraint_solver/constraint_solver.h>

#include "inferred_model.h"

#include "util/error.h"
#include "util/ortools.h"

#include "legacy/action_formulation.h"
#include "legacy/max_transfer_upper_bound.h"
#include "legacy/max_transfer_distribution_upper_bound.h"
#include "legacy/solver_arguments.h"
#include "legacy/cp_solution.h"
#include "legacy/cp_solution_json_writer.h"
#include "minizinc_data_model.h"
#include "minizinc_reader.h"
#include "solution_printer.h"
#include "solver_cp_command.h"

quake::SolverCpCommand::SolverCpCommand(quake::SolverArguments args)
        : args_{std::move(args)} {}

// Possible future improvements:
// - use switch time in strong upper bound

void quake::SolverCpCommand::Run() {
    using namespace operations_research;

    auto model = InferredModel::Load(args_.ModelPath);
    const auto weak_upper_bound = FindWeakUpperBound(model);
    model.TotalKeyRate().SetUpperBound(weak_upper_bound.TotalKeyRate());
    model.StationKeyRate().SetUpperBound(weak_upper_bound.MaxStationKeyRate());

//    const auto strong_lower_bound = FindStrongLowerBound(model);
//    model.TotalKeyRate().SetLowerBound(strong_lower_bound.TotalKeyRate());
//
//    const auto min_station_key_lb = static_cast<int64>(ceil(
//            (static_cast<double>(strong_lower_bound.MaxStationKeyRate())
//             * MaxScalingFactorOrDefault()) / MinScalingFactorOrDefault()));
//    model.StationKeyRate().SetLowerBound(min_station_key_lb);
//
//    FindStrongUpperBound(model);

    // TODO: save multiple solutions
    auto min_actions = model.StationNoDummyCount();
    if (args_.MinJobs) {
        min_actions = std::max(min_actions, args_.MinJobs.value());
    }

    const auto actions_step = args_.JobsStep.value_or(1);
    auto max_actions = min_actions;
    if (args_.MaxJobs) {
        max_actions = std::max(max_actions, args_.MaxJobs.value());
    }

    if (!args_.TimeLimit) {
        LOG(WARNING) << "No time limit has been provided";
    }

    if (!args_.FailureScalingFactor) {
        LOG(WARNING) << "No failure scaling factor has been provided";
    }

    std::vector<CpSolution> solutions;
    const auto repeats = args_.Repeats.value_or(1);
    for (auto current_actions = min_actions; current_actions <= max_actions; current_actions += actions_step) {
        for (auto current_repeat = 0; current_repeat < repeats; ++current_repeat) {
            LOG(INFO) << "Scheduling with: " << current_actions << " jobs, repeat: " << current_repeat;

            Solver restricted_solver("initial-guess-cp-solver");
            ActionFormulation restricted_formulation(&restricted_solver,
                                                     &model,
                                                     current_actions,
                                                     MinScalingFactorOrDefault(),
                                                     MaxScalingFactorOrDefault());

            restricted_formulation.Build();
            const auto restricted_solution = restricted_formulation.Solve(args_.FailureScalingFactor,
                                                                          args_.TimeLimit,
                                                                          args_.PrintSolutions);
            model.CheckConsistency(restricted_solution);

            Solver solver("local-search-solver");
            ActionFormulation formulation(&solver,
                                          &model,
                                          current_actions,
                                          MinScalingFactorOrDefault(),
                                          MaxScalingFactorOrDefault());

            formulation.Build();

            Assignment *const restricted_assignment = formulation.CreateAssignment(restricted_solution);
            CHECK(solver.CheckAssignment(restricted_assignment));

            const auto solution = formulation.SolveWithLocalSearch(restricted_assignment,
                                                                   args_.FailureScalingFactor,
                                                                   args_.TimeLimit,
                                                                   args_.PrintSolutions);
            model.CheckConsistency(solution);

            auto dummy_jobs = 0;
            for (const auto &action : solution.Jobs()) {
                if (action.Duration() == 0) {
                    ++dummy_jobs;
                }
            }

            LOG(INFO) << "Total transferred keys: " << solution.TotalKeyRate() << ", dummy jobs: " << dummy_jobs;
            solutions.push_back(solution);
        }
    }

    std::ofstream output_file;
    output_file.open(args_.OutputPath.string(), std::fstream::out | std::fstream::trunc);
    if (!output_file.good()) {
        throw util::OnFailedSaveOutput(args_.OutputPath);
    }

    CpSolutionJsonWriter<std::ofstream> json_writer{std::move(output_file)};
    json_writer.Write(model, solutions);
    json_writer.Close();
}

quake::CpSolution quake::SolverCpCommand::FindWeakUpperBound(const quake::InferredModel &model) const {
    using namespace operations_research;

    operations_research::Solver solver("find-weak-upper-bound");
    MaxTransferUpperBound upper_bound_model(&solver, &model);
    upper_bound_model.Build();
    const auto solution = upper_bound_model.Solve();
    upper_bound_model.CheckConsistency(solution);
    return solution;
}

quake::CpSolution quake::SolverCpCommand::FindStrongUpperBound(const quake::InferredModel &model) const {
    using namespace operations_research;

    operations_research::Solver solver("find-strong-upper-bound");
    MaxTransferDistributionUpperBound upper_bound_model(&solver,
                                                        &model,
                                                        MinScalingFactorOrDefault(),
                                                        MaxScalingFactorOrDefault());
    upper_bound_model.Build();
    const auto solution = upper_bound_model.Solve();
    upper_bound_model.CheckConsistency(solution);
    return solution;
}

quake::CpSolution quake::SolverCpCommand::FindStrongLowerBound(const quake::InferredModel &model) const {
    using namespace operations_research;

    Solver restricted_solver("restricted-solver");
    ActionFormulation restricted_formulation(&restricted_solver,
                                             &model,
                                             model.StationNoDummyCount(),
                                             MinScalingFactorOrDefault(),
                                             MaxScalingFactorOrDefault());
    restricted_formulation.Build();
    const auto restricted_solution = restricted_formulation.Solve(boost::none, args_.TimeLimit, args_.PrintSolutions);
    model.CheckConsistency(restricted_solution);
    return restricted_solution;
}

int quake::SolverCpCommand::MinScalingFactorOrDefault() const {
    return args_.MinScalingFactor.get_value_or(11);
}

int quake::SolverCpCommand::MaxScalingFactorOrDefault() const {
    return args_.MaxScalingFactor.get_value_or(10);
}
