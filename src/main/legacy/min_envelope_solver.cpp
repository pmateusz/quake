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

#include "min_envelope_solver.h"

#include "objective_function.h"

quake::MinEnvelopeSolver::MinEnvelopeSolver(const quake::InferredModel *model,
                                            int64 min_transfer_size,
                                            const quake::SolverArguments *args)
        : OneStageSolver(model, min_transfer_size, args) {}

quake::ObjectiveFunction quake::MinEnvelopeSolver::BuildObjective(const CpSolution &initial_solution) {
    static const auto SCALING_FACTOR = 1000000000000;

    const auto &initial_buffer = Model()->InitialBuffer();
    auto &solver = Solver();

    // there could be either initial buffer or weight
    operations_research::IntVar *max_envelope = solver.MakeIntVar(0, kint64max);
    for (std::size_t station_index = 1; station_index < StationSize(); ++station_index) {
        solver.AddConstraint(
                solver.MakeGreaterOrEqual(solver.MakeProd(max_envelope, FinalBuffer()[station_index]),
                                          static_cast<int64>(Model()->TransferShare(station_index) * SCALING_FACTOR)
                        /*Model()->InitialBuffer()[station_index]*/));
    }

    operations_research::OptimizeVar *objective = solver.MakeMinimize(max_envelope, 1);
    return {true, objective->Var(), objective};
}
