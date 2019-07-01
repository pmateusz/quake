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

#ifndef QUAKE_ACTION_FORMULATION_H
#define QUAKE_ACTION_FORMULATION_H

#include <boost/date_time.hpp>
#include <boost/optional.hpp>

#include <ortools/constraint_solver/constraint_solveri.h>

#include "inferred_model.h"

namespace quake {

    class CpSolution;

    class ActionFormulation {
    public:
        ActionFormulation(operations_research::Solver *solver,
                          quake::InferredModel const *model,
                          int64 actions,
                          int64 min_multiplier,
                          int64 max_multiplier);

        void Build();

        quake::CpSolution Solve(boost::optional<int> failure_frequency,
                                boost::optional<boost::posix_time::time_duration> time_limit,
                                bool print_solutions);

        quake::CpSolution SolveWithLocalSearch(operations_research::Assignment *assignment,
                                               boost::optional<int> failure_frequency,
                                               boost::optional<boost::posix_time::time_duration> time_limit,
                                               bool print_solutions);

        quake::CpSolution GetSolution() const;

        operations_research::Assignment *CreateAssignment(const quake::CpSolution &solution) const;

    private:
        operations_research::DecisionBuilder *CreateDefaultDecisionBuilder() const;

        std::vector<operations_research::SearchMonitor *> CreateSearchMonitors(boost::optional<int> failure_frequency,
                                                                               boost::optional<boost::posix_time::time_duration> time_limit) const;

        void ListSolutions() const;

        operations_research::Solver *solver_;
        quake::InferredModel const *model_;
        int64 actions_;
        int64 min_multiplier_;
        int64 max_multiplier_;

        std::vector<operations_research::IntVar *> stations_;
        std::vector<operations_research::IntVar *> active_;
        std::vector<operations_research::IntVar *> start_times_;
        std::vector<operations_research::IntExpr *> station_offset_;
        std::vector<operations_research::IntVar *> start_index_;
        std::vector<operations_research::IntVar *> end_index_;
        std::vector<operations_research::IntVar *> reconfiguration_time_;
        std::vector<operations_research::IntVar *> action_transfers_;
        operations_research::IntVar *active_actions_;
        operations_research::IntVar *total_transfer_;
        std::vector<operations_research::IntVar *> station_transfers_;
        operations_research::OptimizeVar *total_transfer_obj_;

        operations_research::SolutionCollector *solution_collector_;
    };
}

#endif //QUAKE_ACTION_FORMULATION_H
