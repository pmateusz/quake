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

#ifndef QUAKE_OBJECTIVE_FUNCTION_H
#define QUAKE_OBJECTIVE_FUNCTION_H

#include <ortools/constraint_solver/constraint_solver.h>

namespace quake {

    struct ObjectiveFunction {
        ObjectiveFunction(bool minimize,
                          operations_research::IntVar *cost_variable,
                          operations_research::OptimizeVar *optimize_variable)
                : Minimize{minimize},
                  CostVariable{cost_variable},
                  OptimizeVariable{optimize_variable} {}

        inline bool IsMinimize() const { return Minimize; }

        inline bool IsMaximize() const { return !Minimize; }

        bool Minimize;
        operations_research::IntVar *CostVariable;
        operations_research::OptimizeVar *OptimizeVariable;
    };
}

#endif //QUAKE_OBJECTIVE_FUNCTION_H
