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

#ifndef QUAKE_SOLUTION_PRINTER_H
#define QUAKE_SOLUTION_PRINTER_H

#include <vector>

#include <ortools/constraint_solver/constraint_solver.h>

namespace quake {

    class SolutionPrinter : public operations_research::SearchMonitor {
    public:
        SolutionPrinter(std::vector<operations_research::IntVar *> observed_variables,
                        operations_research::Solver *solver);

        bool AtSolution() override;

    private:
        std::vector<operations_research::IntVar *> observed_variables_;
        std::size_t observed_variables_size_;
    };
}


#endif //QUAKE_SOLUTION_PRINTER_H
