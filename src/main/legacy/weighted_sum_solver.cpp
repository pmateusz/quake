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

#include "weighted_sum_solver.h"

#include "objective_function.h"

quake::WeightedSumSolver::WeightedSumSolver(const quake::InferredModel *model,
                                            int64 min_transfer_size,
                                            const quake::SolverArguments *args)
        : OneStageSolver(model, min_transfer_size, args) {}

quake::ObjectiveFunction quake::WeightedSumSolver::BuildObjective(const CpSolution &initial_solution) {
    static const int64 MAX_SCALING_FACTOR = 100000000;

    const auto min_rate_scaling_factor = Model()->GetMinRateScalingFactor();

    std::vector<double> local_lambda;
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        CHECK_GT(initial_solution.FinalBuffer(station_index), 0);
        local_lambda.push_back(static_cast<double> (initial_solution.FinalBuffer(station_index))
                               / (Model()->TransferShare(station_index) * min_rate_scaling_factor));
    }

    const auto min_initial_lambda_it = std::min_element(std::cbegin(local_lambda), std::cend(local_lambda));
    const auto min_initial_lambda = static_cast<int64>(floor(*min_initial_lambda_it));

    auto &solver = Solver();
    operations_research::IntVar *lower_envelope = solver.MakeIntVar(min_initial_lambda, kint64max, "lower_envelope");
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        const auto local_lambda_coefficient
                = static_cast<int64>(round(Model()->TransferShare(station_index) * min_rate_scaling_factor));
        solver.AddConstraint(solver.MakeGreaterOrEqual(FinalBuffer()[station_index],
                                                       solver.MakeProd(lower_envelope, local_lambda_coefficient)));
    }

    auto &keys_transferred = KeysTransferred();
    operations_research::IntVar *total_transferred_keys = solver.MakeSum(keys_transferred)->Var();
    operations_research::IntVar *objective_value = solver.MakeSum(solver.MakeProd(lower_envelope, MAX_SCALING_FACTOR),
                                                                  total_transferred_keys)->Var();

    operations_research::OptimizeVar *optimize_var = solver.MakeMaximize(objective_value, 1);
    return {false, objective_value, optimize_var};
}

