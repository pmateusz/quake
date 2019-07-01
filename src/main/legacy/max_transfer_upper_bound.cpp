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

#include <algorithm>

#include "max_transfer_upper_bound.h"

#include "cp_solution.h"

quake::MaxTransferUpperBound::MaxTransferUpperBound(operations_research::Solver *solver,
                                                    quake::InferredModel const *model)
        : solver_{solver},
          model_{model},
          total_transfer_{nullptr},
          total_transfer_obj_{nullptr} {
}

quake::CpSolution quake::MaxTransferUpperBound::GetSolution(operations_research::SolutionCollector const *collector,
                                                            int solution_number) const {
    std::vector<quake::CpSolution::Job> actions;

    for (int64 start_time = 0; start_time < model_->TimeRange();) {
        int64 station_index = collector->Value(solution_number, station_[start_time]);

        int64 end_time = start_time + 1;
        for (; end_time < model_->TimeRange()
               && collector->Value(solution_number, station_[end_time]) == station_index; ++end_time);
        int64 duration = end_time - start_time;
        CHECK_GT(duration, 0);

        int64 key_rate = model_->KeyRateCumulative1d().at(
                static_cast<std::size_t>(model_->CumulativeIndex(station_index, start_time + duration)));
        if (start_time > 0) {
            key_rate -= model_->KeyRateCumulative1d().at(
                    static_cast<std::size_t>(model_->CumulativeIndex(station_index, start_time)));
        }
        CHECK_GE(key_rate, 0);

        actions.emplace_back(start_time, duration, station_index, key_rate);

        start_time = end_time;
    }

    return quake::CpSolution{std::move(actions)};
}

void quake::MaxTransferUpperBound::Build() {
    const auto &key_rate_1d = model_->KeyRate1d();
    const auto time_range = model_->TimeRange();
    const auto stations = model_->StationCount();

    for (auto time_index = 0; time_index < time_range; ++time_index) {
        station_.push_back(solver_->MakeIntVar(1, model_->StationNoDummyCount()));
    }

    const auto minmax_element_it = std::minmax_element(std::cbegin(key_rate_1d), std::cend(key_rate_1d));

    for (int64 time_index = 0; time_index < time_range; ++time_index) {
        std::vector<int64> local_transfer_rates;
        for (int64 station_index = 0; station_index < stations; ++station_index) {
            // this index is different from cumulative, since it does not have the extra slot for zero as duration
            const auto index = static_cast<const size_t>(station_index * time_range + time_index);
            local_transfer_rates.push_back(key_rate_1d.at(index));
        }

#if DEBUG
        for (const auto transfer_rate : local_transfer_rates) {
            CHECK_GE(transfer_rate, *minmax_element_it.first);
            CHECK_LE(transfer_rate, *minmax_element_it.second);
        }
#endif

        transfer_.push_back(solver_->MakeElement(local_transfer_rates, station_[time_index])->Var());

        solver_->AddConstraint(solver_->MakeMemberCt(transfer_[time_index], local_transfer_rates));
    }

    for (auto station_index = 1; station_index < stations; ++station_index) {
        std::vector<operations_research::IntVar *> cost_element;
        for (auto time_index = 0; time_index < time_range; ++time_index) {
            cost_element.push_back(
                    solver_->MakeProd(solver_->MakeIsEqualCstVar(station_[time_index], station_index),
                                      transfer_[time_index])->Var()
            );
        }
        station_transfer_.push_back(solver_->MakeSum(cost_element)->Var());
    }

    DCHECK_EQ(station_.size(), time_range);
    DCHECK_EQ(transfer_.size(), time_range);
    DCHECK_EQ(station_transfer_.size(), model_->StationNoDummyCount());

    total_transfer_ = solver_->MakeSum(transfer_)->Var();
    total_transfer_obj_ = solver_->MakeMaximize(total_transfer_, 1);
}

// TODO: remove
//void quake::MaxTransferUpperBound::BuildStrong(int64 max_multiplier, int64 other_multiplier) {
//    Build();
//
//    if (model_->TotalKeyRate().HasUpperBound()) {
//        solver_->AddConstraint(solver_->MakeLessOrEqual(total_transfer_, model_->TotalKeyRate().UpperBound()));
//    }
//
//    if (model_->TotalKeyRate().HasLowerBound()) {
//        solver_->AddConstraint(
//                solver_->MakeGreaterOrEqual(total_transfer_, model_->TotalKeyRate().LowerBound()));
//    }
//
//    if (model_->StationKeyRate().HasUpperBound()) {
//        for (auto station_index = 0; station_index < model_->StationNoDummyCount(); ++station_index) {
//            solver_->AddConstraint(
//                    solver_->MakeLessOrEqual(station_transfer_[station_index],
//                                             model_->StationKeyRate().UpperBound()));
//        }
//    }
//
//    if (model_->StationKeyRate().HasLowerBound()) {
//        for (auto station_index = 0; station_index < model_->StationNoDummyCount(); ++station_index) {
//            solver_->AddConstraint(
//                    solver_->MakeGreaterOrEqual(station_transfer_[station_index],
//                                                model_->StationKeyRate().LowerBound()));
//        }
//    }
//
//    operations_research::IntExpr *const max_transfer_rate = solver_->MakeMax(station_transfer_);
//    for (auto station_index = 0; station_index < model_->StationNoDummyCount(); ++station_index) {
//        solver_->AddConstraint(solver_->MakeLessOrEqual(solver_->MakeProd(max_transfer_rate, max_multiplier),
//                                                        solver_->MakeProd(station_transfer_[station_index],
//                                                                          other_multiplier)));
//    }
//}

void quake::MaxTransferUpperBound::CheckConsistency(const quake::CpSolution &solution) const {
    for (const auto &action : solution.Jobs()) {
        if (action.Station() == model_->DummyStationIndex()) {
            CHECK_EQ(action.Duration(), 0);
            CHECK_EQ(action.KeysTransferred(), 0);
        } else {
            const auto start_transfer_time = action.Start();
            const auto end_transfer_time = action.Start() + action.Duration();
            const auto &key_rate = model_->KeyRate()[action.Station()];

            const auto action_key_rate = std::accumulate(std::cbegin(key_rate) + start_transfer_time,
                                                         std::cbegin(key_rate) + end_transfer_time,
                                                         static_cast<int64>(0));
            CHECK_EQ(action.KeysTransferred(), action_key_rate);
        }
    }

    int64 total_action_key_rate = 0;
    for (const auto &action : solution.Jobs()) {
        total_action_key_rate += action.KeysTransferred();
    }

    CHECK_EQ(total_action_key_rate, solution.TotalKeyRate());
}

quake::CpSolution quake::MaxTransferUpperBound::Solve() {
    using namespace operations_research;

    DecisionBuilder *const transfer_phase = solver_->MakePhase(transfer_,
                                                               Solver::CHOOSE_FIRST_UNBOUND,
                                                               Solver::ASSIGN_MAX_VALUE);

    DecisionBuilder *const station_phase = solver_->MakePhase(station_,
                                                              Solver::CHOOSE_FIRST_UNBOUND,
                                                              Solver::ASSIGN_MIN_VALUE);

    DecisionBuilder *const main_phase = solver_->Compose(transfer_phase, station_phase);

    VLOG(1) << "Started search for a weak upper bound";

    SolutionCollector *const collector = solver_->MakeLastSolutionCollector();
    collector->Add(total_transfer_);
    collector->Add(transfer_);
    collector->Add(station_transfer_);
    collector->Add(station_);

    solver_->Solve(main_phase, total_transfer_obj_, collector);
    CHECK_EQ(collector->solution_count(), 1);

    const auto solution_number = 0;
    const auto solution = GetSolution(collector, solution_number);

#if DEBUG
    CheckConsistency(solution);

    int64 total_transfer_recorded = 0;
    int64 total_transfer = 0;
    for (auto time_index = 0; time_index < model_->TimeRange(); ++time_index) {
        total_transfer_recorded += collector->Value(solution_number, transfer_[time_index]);
        total_transfer += model_->KeyRate()[collector->Value(solution_number, station_[time_index])][time_index];
    }
    CHECK_EQ(total_transfer, total_transfer_recorded);
    CHECK_EQ(total_transfer, collector->Value(solution_number, total_transfer_));
    CHECK_EQ(solution.TotalKeyRate(), collector->Value(solution_number, total_transfer_));
#endif

    VLOG(1) << "Found weak upper bound " << solution.TotalKeyRate() << " in " << solver_->wall_time() << " ms";

    return solution;
}