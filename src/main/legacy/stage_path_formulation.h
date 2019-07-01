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

#ifndef QUAKE_STAGE_PATH_FORMULATION_H
#define QUAKE_STAGE_PATH_FORMULATION_H

#include <ctype.h>

#include <ortools/constraint_solver/constraint_solver.h>

#include "inferred_model.h"

namespace quake {

    class StagePathFormulation {
    public:
        StagePathFormulation(InferredModel const *model,
                             std::size_t stations,
                             std::size_t begin_index,
                             std::size_t end_index,
                             int64 min_transfer_size);

        void Build(operations_research::Solver *solver);

        void CheckConsistency(int64 solution_number,
                              operations_research::SolutionCollector const *solution_collector) const;

        std::vector<CpSolution::Job> GetJobs(int solution_number,
                                             operations_research::SolutionCollector const *solution_collector) const;

        void Clear();

        inline std::vector<operations_research::IntVar *> &Next() { return next_; }

        inline const std::vector<operations_research::IntVar *> &Next() const { return next_; }

        inline std::vector<operations_research::IntVar *> &StartTime() { return start_time_; }

        inline const std::vector<operations_research::IntVar *> &StartTime() const { return start_time_; }

        inline std::vector<operations_research::IntVar *> &Transfer() { return transfer_; }

        inline const std::vector<operations_research::IntVar *> &Transfer() const { return transfer_; }

        inline operations_research::IntVar *const Transfer(std::size_t index) const { return transfer_[index]; }

        inline std::vector<operations_research::IntVar *> &StartTransferTime() { return start_transfer_time_; }

        inline std::vector<operations_research::IntVar *> &EndTransferTime() { return end_transfer_time_; }

        inline bool IsEndNode(std::size_t node_index) const { return node_index == EndNode(); }

        inline std::size_t BeginStageIndex() const { return begin_stage_index_; }

        inline std::size_t EndStageIndex() const { return end_stage_index_; }

        inline int64 EndNode() const { return stations_; }

        inline int64 StartNode() const { return 0; }

        void SetMinTransferSize(int64 min_transfer_size) { min_transfer_size_ = min_transfer_size; }

    private:
        InferredModel const *model_;

        std::size_t stations_;
        std::size_t begin_stage_index_;
        std::size_t end_stage_index_;

        std::vector<operations_research::IntVar *> next_;
        std::vector<operations_research::IntVar *> active_;
        std::vector<operations_research::IntVar *> start_time_;

        std::vector<operations_research::IntVar *> start_transfer_time_;
        std::vector<operations_research::IntVar *> end_transfer_time_;
        std::vector<operations_research::IntVar *> transfer_;

        int64 min_transfer_size_;
    };
}


#endif //QUAKE_STAGE_PATH_FORMULATION_H
