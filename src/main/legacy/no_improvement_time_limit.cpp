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


#include "no_improvement_time_limit.h"

quake::NoImprovementTimeLimit::NoImprovementTimeLimit(operations_research::Solver *const solver,
                                                      operations_research::IntVar *const objective_var,
                                                      boost::posix_time::time_duration time_limit,
                                                      const bool maximize)
        : SearchLimit(solver),
          prototype_(new operations_research::Assignment(solver)),
          time_limit_(std::move(time_limit)),
          maximize_(maximize),
          limit_reached_(false) {
    if (maximize_) {
        best_result_ = kint64min;
    } else {
        best_result_ = kint64max;
    }

    CHECK_NOTNULL(objective_var);
    prototype_->AddObjective(objective_var);
}

void quake::NoImprovementTimeLimit::Init() {
    best_solution_time_ = solver()->wall_time();
    limit_reached_ = false;
    has_solution_ = false;
    if (maximize_) {
        best_result_ = kint64min;
    } else {
        best_result_ = kint64max;
    }
}

//  Returns true if limit is reached, false otherwise.
bool quake::NoImprovementTimeLimit::Check() {
    if (!limit_reached_) {
        limit_reached_ = is_time_limit_crossed(solver()->wall_time());
    }
    return limit_reached_;
}

bool quake::NoImprovementTimeLimit::AtSolution() {
    prototype_->Store();

    const operations_research::IntVar *objective = prototype_->Objective();
    has_solution_ = true;

    const auto current_wall_time = solver()->wall_time();
    if (maximize_ && objective->Min() > best_result_) {
        best_result_ = objective->Min();
        best_solution_time_ = current_wall_time;
    } else if (!maximize_ && objective->Max() < best_result_) {
        best_result_ = objective->Max();
        best_solution_time_ = current_wall_time;
    }

    limit_reached_ = is_time_limit_crossed(current_wall_time);
    return true;
}

void quake::NoImprovementTimeLimit::Copy(const SearchLimit *const limit) {
    const auto *const copy_limit = reinterpret_cast<const NoImprovementTimeLimit *const>(limit);

    best_result_ = copy_limit->best_result_;
    best_solution_time_ = copy_limit->best_solution_time_;
    maximize_ = copy_limit->maximize_;
    limit_reached_ = copy_limit->limit_reached_;

}

// Allocates a clone of the limit
operations_research::SearchLimit *quake::NoImprovementTimeLimit::MakeClone() const {
    // we don't to copy the variables
    return solver()->RevAlloc(
            new NoImprovementTimeLimit(solver(), prototype_->Objective(), time_limit_, maximize_));
}

std::string quake::NoImprovementTimeLimit::DebugString() const {
    std::stringstream msg;
    msg << "NoImprovementTimeLimit(crossed = " << limit_reached_ << ")";
    return msg.str();
}