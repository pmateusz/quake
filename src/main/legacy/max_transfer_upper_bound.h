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

#ifndef QUAKE_MAX_TRANSFER_UPPER_BOUND_H
#define QUAKE_MAX_TRANSFER_UPPER_BOUND_H

#include <ortools/constraint_solver/constraint_solveri.h>

#include "inferred_model.h"

namespace quake {

    class CpSolution;

    class MaxTransferUpperBound {
    public:
        MaxTransferUpperBound(operations_research::Solver *solver, quake::InferredModel const *model);

        virtual void Build();

        CpSolution Solve();

        CpSolution GetSolution(operations_research::SolutionCollector const *collector, int solution_number) const;

        void CheckConsistency(const CpSolution &solution) const;

        inline const std::vector<operations_research::IntVar *> &station() const { return station_; }

        inline const std::vector<operations_research::IntVar *> &transfer() const { return transfer_; }

        inline const std::vector<operations_research::IntVar *> &station_transfer() const { return station_transfer_; }

        inline operations_research::IntVar *total_transfer() {
            return total_transfer_;
        }

        inline operations_research::OptimizeVar *total_transfer_obj() {
            return total_transfer_obj_;
        }

    protected:
        std::vector<operations_research::IntVar *> station_;
        std::vector<operations_research::IntVar *> transfer_;
        std::vector<operations_research::IntVar *> station_transfer_;

        operations_research::IntVar *total_transfer_;
        operations_research::OptimizeVar *total_transfer_obj_;

        operations_research::Solver *solver_;
        quake::InferredModel const *model_;
    };
}

#endif //QUAKE_MAX_TRANSFER_UPPER_BOUND_H
