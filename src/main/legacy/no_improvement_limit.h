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


#ifndef QUAKE_NO_IMPROVEMENT_LIMIT_H
#define QUAKE_NO_IMPROVEMENT_LIMIT_H

#include <string>

#include <ortools/constraint_solver/constraint_solver.h>

namespace quake {

    class NoImprovementLimit : public operations_research::SearchLimit {
    public:
        NoImprovementLimit(operations_research::Solver *solver,
                           operations_research::IntVar *objective_var,
                           int solution_nbr_tolerance,
                           bool minimize = true);

        void Init() override;

        bool Check() override;

        bool AtSolution() override;

        void Copy(const operations_research::SearchLimit *limit) override;

        operations_research::SearchLimit *MakeClone() const override;

        std::string DebugString() const override;

    private:
        operations_research::Solver *const solver_;
        int64 best_result_;
        int solution_nbr_tolerance_;
        bool minimize_;
        bool limit_reached_;
        int nbr_solutions_with_no_better_obj_;
        std::unique_ptr<operations_research::Assignment> prototype_;
    };
}

#endif //QUAKE_NO_IMPROVEMENT_LIMIT_H
