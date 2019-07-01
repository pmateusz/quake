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

#include "gurobi.h"

std::ostream &quake::util::operator<<(std::ostream &out, SolverStatus status) {
    switch (status) {
        case SolverStatus::Loaded:
            out << "Loaded";
            return out;
        case SolverStatus::Optimal:
            out << "Optimal";
            return out;
        case SolverStatus::Infeasible:
            out << "Infeasible";
            return out;
        case SolverStatus::InfiniteOrUnbounded:
            out << "InfiniteOrUnbounded";
            return out;
        case SolverStatus::Unbounded:
            out << "Unbounded";
            return out;
        case SolverStatus::Cutoff:
            out << "Cutoff";
            return out;
        case SolverStatus::IterationLimit:
            out << "IterationLimit";
            return out;
        case SolverStatus::NodeLimit:
            out << "NodeLimit";
            return out;
        case SolverStatus::TimeLimit:
            out << "TimeLimit";
            return out;
        case SolverStatus::SolutionLimit:
            out << "SolutionLimit";
            return out;
        case SolverStatus::Interrupted:
            out << "Interrupted";
            return out;
        case SolverStatus::Numeric:
            out << "Numeric";
            return out;
        case SolverStatus::Suboptimal:
            out << "Suboptimal";
            return out;
        case SolverStatus::InProgress:
            out << "InProgress";
            return out;
        case SolverStatus::UserObjectiveLimit:
            out << "UserObjectiveLimit";
            return out;
    }
}

extern const double quake::util::ZERO_THRESHOLD = 0.00001;
extern const double quake::util::ONE_THRESHOLD = 0.99999;

bool quake::util::IsActive(const GRBVar &var) {
    return IsActive(var.get(GRB_DoubleAttr_X));
}

bool quake::util::IsActive(double value) {
    CHECK(value <= ZERO_THRESHOLD || value >= ONE_THRESHOLD);
    return value >= ONE_THRESHOLD;
}
