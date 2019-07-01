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

#ifndef QUAKE_ONE_STAGE_SOLVER_H
#define QUAKE_ONE_STAGE_SOLVER_H

#include "solver_arguments.h"
#include "inventory_formulation.h"

#include <boost/config.hpp>
#include <boost/optional.hpp>

#include <ortools/constraint_solver/constraint_solver.h>

namespace quake {

    class ObjectiveFunction;

    class OneStageSolver : public InventoryFormulation {
    public:
        OneStageSolver(InferredModel const *model, int64 min_transfer_size, quake::SolverArguments const *args);

        boost::optional<CpSolution> Solve(const CpSolution &initial_guess);

    protected:

        virtual ObjectiveFunction BuildObjective(const CpSolution &initial_solution) = 0;

        inline operations_research::Solver &Solver() { return solver_; }

        inline const std::vector<operations_research::IntVar *> &KeysTransferred() const { return keys_transferred_; }

        inline const std::vector<operations_research::IntVar *> &FinalBuffer() const { return final_buffer_; }

    private:
        boost::optional<CpSolution> Solve(
                const CpSolution &initial_solution, operations_research::Assignment *assignment);

        operations_research::Assignment *const BuildAssignment(const CpSolution &solution);

        SolverArguments const *args_;
        operations_research::Solver solver_;
        std::vector<operations_research::IntVar *> keys_transferred_;
        std::vector<operations_research::IntVar *> final_buffer_;
    };
}


#endif //QUAKE_ONE_STAGE_SOLVER_H
