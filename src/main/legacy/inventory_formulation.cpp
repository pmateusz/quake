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

#include "inventory_formulation.h"
#include "no_improvement_time_limit.h"
#include "objective_function.h"
#include "solver_arguments.h"

quake::InventoryFormulation::InventoryFormulation(const InferredModel *model)
        : InventoryFormulation(model, model->IsWeatherAdjusted() ? 100 : 0) {}

quake::InventoryFormulation::InventoryFormulation(const quake::InferredModel *model, int64 min_transfer_size)
        : model_{model} {
    const auto stages = model_->InferStages();
    const auto stages_size = stages.size();
    for (auto stage_index = 0; stage_index < stages_size - 1; ++stage_index) {
        stage_path_formulations_.emplace_back(model_,
                                              model_->StationCount(),
                                              stages[stage_index].OffsetIndex,
                                              stages[stage_index + 1].OffsetIndex,
                                              min_transfer_size);
    }
    stage_path_formulations_.emplace_back(model_,
                                          model_->StationCount(),
                                          stages[stages_size - 1].OffsetIndex,
                                          model_->TimeOffset().size(),
                                          min_transfer_size);

#if DEBUG
    const auto &key_rate = model_->KeyRate1d();
    const auto &key_rate_cumulative = model_->KeyRateCumulative1d();
    for (auto stage_index = 0; stage_index < stages_size; ++stage_index) {
        const auto begin_index = stage_path_formulations_[stage_index].BeginStageIndex();
        const auto end_index = stage_path_formulations_[stage_index].EndStageIndex();

        for (auto station_index = 0; station_index < model_->StationCount(); ++station_index) {
            const auto begin = model_->Index(station_index, begin_index);
            const auto end = model_->Index(station_index, end_index);
            CHECK_LE(begin, key_rate.size());
            CHECK_LE(end, key_rate.size());

            const auto cumulative_begin = model_->CumulativeIndex(station_index, begin_index);
            const auto cumulative_end = model_->CumulativeIndex(station_index, end_index);
            CHECK_LE(cumulative_begin, key_rate_cumulative.size());
            CHECK_LE(cumulative_end, key_rate_cumulative.size());
        }
    }
#endif
}

static const auto NO_BETTER_SOLUTION_TIME_LIMIT = boost::posix_time::seconds(60);

void quake::InventoryFormulation::AppendSearchMonitors(operations_research::Solver *solver,
                                                       const ObjectiveFunction &objective_function,
                                                       const SolverArguments &args,
                                                       std::vector<operations_research::SearchMonitor *> &search_monitors) {
    if (args.FailureScalingFactor) {
        // TODO2: select a better restart option
        // search_monitors.push_back(solver->MakeLubyRestart(failure_frequency.get()));
        search_monitors.push_back(solver->MakeConstantRestart(args.FailureScalingFactor.get()));
    }
    if (args.TimeLimit) {
        search_monitors.push_back(solver->MakeTimeLimit(args.TimeLimit.get().total_milliseconds()));
    } else {
        search_monitors.push_back(
                solver->RevAlloc(new NoImprovementTimeLimit(solver,
                                                            objective_function.CostVariable,
                                                            NO_BETTER_SOLUTION_TIME_LIMIT,
                                                            objective_function.IsMaximize())));
    }
}
