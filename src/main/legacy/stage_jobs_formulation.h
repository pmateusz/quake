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

#ifndef QUAKE_STAGE_JOBS_FORMULATION_H
#define QUAKE_STAGE_JOBS_FORMULATION_H

#include <ortools/constraint_solver/constraint_solver.h>

#include <vector>

#include "cp_solution.h"
#include "inferred_model.h"

namespace quake {

    class StageJobsFormulation {
    public:
        StageJobsFormulation(InferredModel const *model,
                             std::size_t begin_index,
                             std::size_t end_index,
                             int64 min_jobs,
                             int64 max_jobs);

        void Build(operations_research::Solver *solver);

        void CheckConsistency(int64 solution_number,
                              operations_research::SolutionCollector const *solution_collector) const;

        std::vector<CpSolution::Job> GetJobs(int solution_number,
                                             operations_research::SolutionCollector const *solution_collector) const;

        void Clear();

        inline std::size_t BeginStageIndex() const { return begin_stage_index_; }

        inline std::size_t EndStageIndex() const { return end_stage_index_; }

        inline const std::vector<operations_research::IntVar *> &Station() const { return station_; }

        inline const std::vector<operations_research::IntVar *> &StartTime() const { return start_time_; }

        inline const std::vector<operations_research::IntVar *> &JobStartIndex() const {
            return job_transfer_start_index_;
        }

        inline const std::vector<operations_research::IntVar *> &JobEndIndex() const {
            return job_transfer_end_index_;
        }

        inline const std::vector<operations_research::IntVar *> &JobTransfer() const { return job_transfer_; }

        inline const std::vector<operations_research::IntVar *> &StationTransfer() const { return station_transfer_; }

    private:
        InferredModel const *model_;

        std::size_t begin_stage_index_;
        std::size_t end_stage_index_;
        int64 min_jobs_;
        int64 max_jobs_;

        std::vector<operations_research::IntVar *> station_;
        std::vector<operations_research::IntVar *> active_;
        std::vector<operations_research::IntVar *> start_time_;
        std::vector<operations_research::IntVar *> reconfiguration_time_;

        std::vector<operations_research::IntVar *> station_offset_;
        std::vector<operations_research::IntVar *> job_transfer_start_index_;
        std::vector<operations_research::IntVar *> job_transfer_end_index_;
        std::vector<operations_research::IntVar *> job_transfer_;
        std::vector<operations_research::IntVar *> station_transfer_;
    };
}

#endif //QUAKE_STAGE_JOBS_FORMULATION_H
