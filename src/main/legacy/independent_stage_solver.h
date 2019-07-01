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

#ifndef QUAKE_INDEPENDENTSTAGESOLVER_H
#define QUAKE_INDEPENDENTSTAGESOLVER_H

#include "inventory_formulation.h"

namespace quake {

    class IndependentStageSolver : public InventoryFormulation {
    public:
        explicit IndependentStageSolver(InferredModel const *model);

        CpSolution Solve(const SolverArguments &args);

    private:
        std::vector<quake::CpSolution::Job> SolveIndependentStage(const std::string &problem_name,
                                                                  StagePathFormulation &stage,
                                                                  const SolverArguments &args);
    };
}

#endif //QUAKE_INDEPENDENTSTAGESOLVER_H
