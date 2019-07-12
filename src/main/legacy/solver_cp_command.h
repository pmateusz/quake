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

#ifndef QUAKE_SOLVE_CP_COMMAND_H
#define QUAKE_SOLVE_CP_COMMAND_H

#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <ortools/base/basictypes.h>

#include "legacy/solver_arguments.h"

#include "bounded_value.h"
#include "command.h"
#include "minizinc_data_model.h"

namespace quake {

    class InferredModel;

    class CpSolution;

    class SolverCpCommand : public quake::Command {
    public:
        explicit SolverCpCommand(SolverArguments args);

        void Run() override;

    private:
        InferredModel LoadModel() const;

        int MinScalingFactorOrDefault() const;

        int MaxScalingFactorOrDefault() const;

        CpSolution FindWeakUpperBound(const InferredModel &model) const;

        CpSolution FindStrongUpperBound(const InferredModel &model) const;

        CpSolution FindStrongLowerBound(const InferredModel &model) const;

        SolverArguments args_;
    };
}


#endif //QUAKE_SOLVE_CP_COMMAND_H
