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

#include "max_envelope_solver.h"

#include "objective_function.h"

quake::MaxEnvelopeSolver::MaxEnvelopeSolver(const quake::InferredModel *model, int64 min_transfer_size,
                                            const quake::SolverArguments *args)
        : OneStageSolver(model, min_transfer_size, args) {}

quake::ObjectiveFunction quake::MaxEnvelopeSolver::BuildObjective(const CpSolution &initial_solution) {
    const auto &initial_buffer = Model()->InitialBuffer();
    const auto min_rate_scaling_factor = Model()->GetMinRateScalingFactor();
    auto &solver = Solver();

    static const auto SCALING_FACTOR = 1000000000;

    operations_research::IntVar *lower_envelope = solver.MakeIntVar(0, kint64max);
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        const auto local_envelope_factor = static_cast<int64>(
                round(Model()->TransferShare(station_index) * min_rate_scaling_factor));

        solver.AddConstraint(
                solver.MakeLessOrEqual(
                        solver.MakeProd(lower_envelope, local_envelope_factor),
                        solver.MakeProd(FinalBuffer()[station_index],
                                        SCALING_FACTOR)));
    }

    operations_research::OptimizeVar *objective = solver.MakeMaximize(lower_envelope, 1);
    return {true, objective->Var(), objective};
}
