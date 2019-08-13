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

#ifndef QUAKE_GUROBI_H
#define QUAKE_GUROBI_H

#include <iostream>

#include <gurobi_c++.h>

#include <glog/logging.h>

namespace quake {

    namespace util {

        enum class SolverStatus {
            Loaded = 1,
            Optimal,
            Infeasible,
            InfiniteOrUnbounded,
            Unbounded,
            Cutoff,
            IterationLimit,
            NodeLimit,
            TimeLimit,
            SolutionLimit,
            Interrupted,
            Numeric,
            Suboptimal,
            InProgress,
            UserObjectiveLimit
        };

        extern const double ZERO_THRESHOLD;
        extern const double ONE_THRESHOLD;

        bool IsActive(const GRBVar &var);

        bool IsActive(double value);

        std::ostream &operator<<(std::ostream &out, SolverStatus status);

        std::vector<GRBVar> CreateVarVector(GRBModel &model,
                                            std::size_t size,
                                            double lower_bound, double upper_bound, const std::string &prefix);

        std::vector<std::vector<GRBVar>> CreateVarMatrix(GRBModel &model,
                                                         std::size_t rows, std::size_t columns,
                                                         double lower_bound, double upper_bound, const std::string &prefix);
    }
}

#endif //QUAKE_GUROBI_H
