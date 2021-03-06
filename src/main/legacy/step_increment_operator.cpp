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

#include "step_increment_operator.h"

quake::StepIncrementOperator::StepIncrementOperator(const std::vector<operations_research::IntVar *> &vars)
        : IntVarLocalSearchOperator(vars),
          current_var_{0},
          current_step_{0},
          max_step_{0} {}

void quake::StepIncrementOperator::OnStart() {
    current_var_ = 0;
    current_step_ = 0;
    max_step_ = 0;

    if (vars_.size() > 1) {
        max_step_ = Value(1) - Value(0);
    }
}

bool quake::StepIncrementOperator::MakeOneNeighbor() {
    if (!Increment()) {
        return false;
    }

    SetValue(current_var_, Value(current_var_) + current_step_);
    return true;
}

bool quake::StepIncrementOperator::Increment() {
    if (++current_step_ > max_step_) {
        current_step_ = 1;
        if (++current_var_ < vars_.size() - 1) {
            max_step_ = Value(current_var_ + 1) - Value(current_var_);
            return true;
        }
        max_step_ = 0;
        return false;
    }
    return true;
}
