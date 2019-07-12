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

#ifndef QUAKE_SOLVER_INVENTORY_CP_COMMAND_H
#define QUAKE_SOLVER_INVENTORY_CP_COMMAND_H

#include "command.h"

#include "legacy/solver_arguments.h"

namespace quake {

    class CpSolution;

    class InferredModel;

    class SolverInventoryCpCommand : public quake::Command {
    public:
        explicit SolverInventoryCpCommand(SolverArguments args);

        void Run() override;

    private:
        CpSolution ComputeOrLoadInitialGuess(const InferredModel &model) const;

        void Save(const InferredModel &model, const CpSolution &solution) const;

        SolverArguments args_;
    };
}


#endif //QUAKE_SOLVER_INVENTORY_CP_COMMAND_H
