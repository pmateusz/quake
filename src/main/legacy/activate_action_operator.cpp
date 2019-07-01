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

#include "activate_action_operator.h"

quake::ActivateActionOperator::ActivateActionOperator(const std::vector<operations_research::IntVar *> &vars,
                                                      int64 min_station,
                                                      int64 max_station)
        : IntVarLocalSearchOperator(vars),
          min_station_{min_station},
          max_station_{max_station} {}

bool quake::ActivateActionOperator::MakeOneNeighbor() {
    if (!Increment()) {
        return false;
    }

    SetValue(current_var_, next_value_);
    return true;
}

bool quake::ActivateActionOperator::Increment() {
    if (current_var_ >= vars_.size()) {
        return false;
    }

    if (++next_value_ > max_station_) {
        if (Value(current_var_) == 0 || ++current_var_ >= vars_.size()) {
            return false;
        }
        next_value_ = min_station_;
    }
    return true;
}

void quake::ActivateActionOperator::OnStart() {
    current_var_ = 0;
    next_value_ = min_station_ - 1;

    for (; current_var_ < vars_.size() && Value(current_var_) != 0; ++current_var_);
}
