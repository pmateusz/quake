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

#ifndef QUAKE_NO_IMPROVEMENT_TIME_LIMIT_H
#define QUAKE_NO_IMPROVEMENT_TIME_LIMIT_H

#include <memory>
#include <string>

#include <boost/date_time.hpp>

#include <ortools/constraint_solver/constraint_solver.h>

namespace quake {

    class NoImprovementTimeLimit : public operations_research::SearchLimit {
    public:
        NoImprovementTimeLimit(operations_research::Solver *solver,
                               operations_research::IntVar *objective_var,
                               boost::posix_time::time_duration time_limit,
                               bool minimize = true);

        void Init() override;

        bool Check() override;

        bool AtSolution() override;

        void Copy(const operations_research::SearchLimit *limit) override;

        operations_research::SearchLimit *MakeClone() const override;

        std::string DebugString() const override;

    private:
        inline bool is_time_limit_crossed(int64 wall_time) const noexcept {
            return has_solution_ && (wall_time - best_solution_time_) > time_limit_.total_milliseconds();
        }

        int64 best_result_;
        int64 best_solution_time_;

        bool maximize_;
        bool has_solution_;
        bool limit_reached_;
        boost::posix_time::time_duration time_limit_;
        std::unique_ptr<operations_research::Assignment> prototype_;
    };
}


#endif //QUAKE_NO_IMPROVEMENT_TIME_LIMIT_H
