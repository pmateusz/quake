// Copyright 2011-2014 Google
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "no_improvement_limit.h"

quake::NoImprovementLimit::NoImprovementLimit(operations_research::Solver *const solver,
                                              operations_research::IntVar *const objective_var,
                                              const int solution_nbr_tolerance,
                                              const bool minimize) :
        SearchLimit(solver),
        solver_(solver),
        prototype_(new operations_research::Assignment(solver_)),
        solution_nbr_tolerance_(solution_nbr_tolerance),
        nbr_solutions_with_no_better_obj_(0),
        minimize_(minimize),
        limit_reached_(false) {
    if (minimize_) {
        best_result_ = kint64max;
    } else {
        best_result_ = kint64min;
    }

    CHECK_NOTNULL(objective_var);
    prototype_->AddObjective(objective_var);
}

void quake::NoImprovementLimit::Init() {
    nbr_solutions_with_no_better_obj_ = 0;
    limit_reached_ = false;
    if (minimize_) {
        best_result_ = kint64max;
    } else {
        best_result_ = kint64min;
    }
}

//  Returns true if limit is reached, false otherwise.
bool quake::NoImprovementLimit::Check() {
    VLOG(2) << "NoImprovementLimit's limit reached? " << limit_reached_;

    return limit_reached_;
}

bool quake::NoImprovementLimit::AtSolution() {
    ++nbr_solutions_with_no_better_obj_;

    prototype_->Store();

    const operations_research::IntVar *objective = prototype_->Objective();

    if (minimize_ && objective->Min() < best_result_) {
        best_result_ = objective->Min();
        nbr_solutions_with_no_better_obj_ = 0;
    } else if (!minimize_ && objective->Max() > best_result_) {
        best_result_ = objective->Max();
        nbr_solutions_with_no_better_obj_ = 0;
    }

    if (nbr_solutions_with_no_better_obj_ > solution_nbr_tolerance_) {
        limit_reached_ = true;
    }
    return true;
}

void quake::NoImprovementLimit::Copy(const SearchLimit *const limit) {
    const auto *const copy_limit = reinterpret_cast<const NoImprovementLimit *const>(limit);

    best_result_ = copy_limit->best_result_;
    solution_nbr_tolerance_ = copy_limit->solution_nbr_tolerance_;
    minimize_ = copy_limit->minimize_;
    limit_reached_ = copy_limit->limit_reached_;
    nbr_solutions_with_no_better_obj_ = copy_limit->nbr_solutions_with_no_better_obj_;
}

// Allocates a clone of the limit
operations_research::SearchLimit *quake::NoImprovementLimit::MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(
            new NoImprovementLimit(solver_, prototype_->Objective(), solution_nbr_tolerance_, minimize_));
}

std::string quake::NoImprovementLimit::DebugString() const {
    std::stringstream msg;
    msg << "NoImprovementLimit(crossed = " << limit_reached_ << ")";
    return msg.str();
}