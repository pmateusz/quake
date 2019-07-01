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

#ifndef QUAKE_INVENTORY_FORMULATION_H
#define QUAKE_INVENTORY_FORMULATION_H

#include "inferred_model.h"
#include "stage_path_formulation.h"

namespace quake {

    class SolverArguments;

    class ObjectiveFunction;

    class InventoryFormulation {
    public:
        explicit InventoryFormulation(InferredModel const *model);

        InventoryFormulation(InferredModel const *model, int64 min_transfer_size);

    protected:
        inline InferredModel const *Model() const { return model_; }

        inline std::vector<StagePathFormulation> &Stages() { return stage_path_formulations_; }

        inline std::size_t StationSize() const { return static_cast<size_t>(model_->StationCount()); }

        void AppendSearchMonitors(operations_research::Solver *solver,
                                  const ObjectiveFunction &objective_function,
                                  const SolverArguments &args,
                                  std::vector<operations_research::SearchMonitor *> &search_monitors);

    private:
        InferredModel const *model_;
        std::vector<StagePathFormulation> stage_path_formulations_;
    };
}


#endif //QUAKE_INVENTORY_FORMULATION_H
