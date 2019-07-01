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

#ifndef QUAKE_LNSPATHSTAGEOPERATOR_H
#define QUAKE_LNSPATHSTAGEOPERATOR_H

#include <ortools/base/random.h>

#include <ortools/constraint_solver/constraint_solver.h>
#include <ortools/constraint_solver/constraint_solveri.h>

namespace quake {

    class LNSPathStageOperator : public operations_research::BaseLns {
    public:
        LNSPathStageOperator(const std::vector<std::vector<operations_research::IntVar *> > &next,
                             const std::vector<std::vector<operations_research::IntVar *> > &start);

        void InitFragments() override;

        bool NextFragment() override;

    private:
        static std::vector<operations_research::IntVar *>
        Flatten(std::vector<std::vector<operations_research::IntVar *> > vars);

        bool ReleaseInactive(std::size_t path);

        bool ReleaseChain(std::size_t path, std::size_t start_node, std::size_t max_length);

        void ReleaseNode(std::size_t path, std::size_t node);

        void NextIteration();

        int64 Next(std::size_t path, std::size_t node);

        int64 Start(std::size_t path, std::size_t node);

        std::size_t paths_;
        int64 min_node_;
        int64 max_node_;
        std::vector<std::size_t> next_offset_;
        std::vector<std::size_t> start_offset_;
        std::vector<std::size_t> size_;

        int64 current_node_;
        std::size_t current_path_;
        std::size_t current_inner_path_;

        bool first_run_;
        std::vector<std::vector<std::size_t> > initial_path_location_;


        std::vector<std::vector<std::size_t> > inactive_node_;
        std::vector<std::size_t> end_node_;
    };
}


#endif //QUAKE_LNSPATHSTAGEOPERATOR_H
