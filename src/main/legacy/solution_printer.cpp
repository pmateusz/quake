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

#include "solution_printer.h"

#include <sstream>

#include <glog/logging.h>

#include "util/ortools.h"

quake::SolutionPrinter::SolutionPrinter(std::vector<operations_research::IntVar *> observed_variables,
                                        operations_research::Solver *const solver)

        : operations_research::SearchMonitor(solver),
          observed_variables_(std::move(observed_variables)),
          observed_variables_size_(observed_variables_.size()) {}

bool quake::SolutionPrinter::AtSolution() {
    if (!observed_variables_.empty()) {
        std::stringstream log_line;

        log_line << util::GetHumanFriendlyValue(observed_variables_[0]);
        for (auto variable_index = 1; variable_index < observed_variables_size_; ++variable_index) {
            log_line << ", " << util::GetHumanFriendlyValue(observed_variables_[variable_index]);
        }

        LOG(INFO) << log_line.str();
    }

    return SearchMonitor::AtSolution();
}


