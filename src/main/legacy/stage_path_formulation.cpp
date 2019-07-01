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

#include "stage_path_formulation.h"

#include <vector>

#include <glog/logging.h>

#include <ortools/constraint_solver/constraint_solver.h>

quake::StagePathFormulation::StagePathFormulation(const quake::InferredModel *model,
                                                  std::size_t stations,
                                                  std::size_t begin_index,
                                                  std::size_t end_index,
                                                  int64 min_transfer_size)
        : model_{model},
          stations_{stations},
          begin_stage_index_{begin_index},
          end_stage_index_{end_index},
          min_transfer_size_{min_transfer_size} {}

void quake::StagePathFormulation::Build(operations_research::Solver *solver) {
    const auto START_NODE = 0;
    const auto END_NODE = stations_;

    const int64 min_time = begin_stage_index_;
    const int64 max_time = end_stage_index_;
    CHECK_LE(min_time, max_time);

    // --> define path
    solver->MakeIntVarArray(stations_, 0, stations_, &next_);
    solver->MakeBoolVarArray(stations_, &active_);
    for (auto next_index = 0; next_index < stations_; ++next_index) {
        next_[next_index]->RemoveValue(START_NODE);
        solver->AddConstraint(solver->MakeIsDifferentCstCt(next_[next_index], next_index, active_[next_index]));
    }

    solver->AddConstraint(solver->MakeAllDifferent(next_, false));
    solver->AddConstraint(solver->MakeNoCycle(next_, active_));

    // --> define start times
    solver->MakeIntVarArray(stations_ + 1, min_time, max_time, &start_time_);

    // --> first start time is zero
    solver->AddConstraint(solver->MakeEquality(start_time_[START_NODE], min_time));
    solver->AddConstraint(solver->MakeEquality(solver->MakeElement(start_time_, next_[START_NODE]), min_time));

    // --> last start is max_time
    solver->AddConstraint(solver->MakeEquality(start_time_[END_NODE], max_time));

    // if it is impossible to transfer any key make the station inactive
    for (auto station_index = 1; station_index < model_->StationCount(); ++station_index) {
        const auto weather_callback = model_->GetWeatherAdjustedCumulativeKeyRate(station_index);
        if (weather_callback(max_time) - weather_callback(min_time) == 0) {
            next_[station_index]->SetValue(station_index);
            active_[station_index]->SetValue(0);
            start_time_[station_index]->SetValue(max_time);
        }
    }

    // --> declare start transfer to enforce strong boundaries on the maximum number of keys possible to send
    start_transfer_time_.push_back(start_time_[0]);
    for (auto station_index = 1; station_index < stations_; ++station_index) {
        start_transfer_time_.push_back(solver->MakeIntVar(min_time, max_time));
    }

    // --> define start transfer
    for (auto station_index = 1; station_index < stations_; ++station_index) {
        solver->AddConstraint(
                solver->MakeEquality(start_transfer_time_[station_index],
                                     solver->MakeMin(
                                             solver->MakeSum(start_time_[station_index],
                                                             solver->MakeElement(model_->ReconfigurationTime(),
                                                                                 start_time_[station_index])),
                                             max_time)));
    }

    // --> define end transfer
    end_transfer_time_.push_back(start_transfer_time_[0]);
    for (auto station_index = 1; station_index < stations_; ++station_index) {
        end_transfer_time_.push_back(solver->MakeElement(start_time_, next_[station_index])->Var());
    }

    // --> start times at next nodes are increasing
    // --> last start index is less than end index
    for (auto station_index = 0; station_index < stations_; ++station_index) {
        solver->AddConstraint(
                solver->MakeLessOrEqual(start_time_[station_index], end_transfer_time_[station_index]));
        solver->AddConstraint(
                solver->MakeLessOrEqual(start_time_[station_index], start_transfer_time_[station_index]));
        solver->AddConstraint(
                solver->MakeLessOrEqual(start_transfer_time_[station_index], end_transfer_time_[station_index]));
    }

    // --> start times that belong to a regular station are offset by switch time
    for (auto station_index = 1; station_index < stations_; ++station_index) {
        solver->AddConstraint(
                solver->MakeEquality(active_[station_index],
                                     solver->MakeIsLessVar(
                                             start_transfer_time_[station_index],
                                             end_transfer_time_[station_index])));
    }

    // --> start times of a dummy station is at the end of time range
    for (auto station_index = 0; station_index < stations_; ++station_index) {
        solver->AddConstraint(
                solver->MakeEquality(active_[station_index],
                                     solver->MakeIsDifferentCstVar(start_time_[station_index], max_time)));
    }

    // --> station transfer
    for (auto station_index = 0; station_index < stations_; ++station_index) {
        const auto station_callback = model_->GetWeatherAdjustedCumulativeKeyRate(station_index);
        transfer_.push_back(
                solver->MakeDifference(
                        solver->MakeElement(station_callback, end_transfer_time_[station_index]->Var()),
                        solver->MakeElement(station_callback, start_transfer_time_[station_index]->Var())
                )->Var());
    }

    // --> transfer is scaled up by 100 so the shortest possible transfer should deliver more than 100 keys
    if (min_transfer_size_ > 0) {
        for (auto station_index = 1; station_index < stations_; ++station_index) {
            solver->AddConstraint(
                    solver->MakeEquality(solver->MakeIsGreaterCstVar(transfer_[station_index], 0),
                                         solver->MakeIsGreaterCstVar(transfer_[station_index], min_transfer_size_)));
        }
    }

    // --> transfer is positive for a regular station
    for (auto station_index = 1; station_index < stations_; ++station_index) {
        solver->AddConstraint(
                solver->MakeEquality(active_[station_index], solver->MakeIsGreaterCstVar(transfer_[station_index], 0)));
    }

    // --> transfer is zero for the depot node
    transfer_[0]->SetValue(0);
}

std::vector<quake::CpSolution::Job> quake::StagePathFormulation::GetJobs(int solution_number,
                                                                         operations_research::SolutionCollector const *solution_collector) const {
    static const auto SOLUTION_NUMBER = 0;

    std::vector<int64> next_values;
    const auto next_size = next_.size();
    for (auto next_index = 0; next_index < next_size; ++next_index) {
        next_values.push_back(solution_collector->Value(SOLUTION_NUMBER, next_[next_index]));
    }

    if (next_values.empty()) {
        return std::vector<CpSolution::Job>();
    }

    std::vector<int64> path_station;
    std::vector<int64> path_start_time;
    std::vector<int64> path_transfer;
    std::size_t current_node_index = next_values[0];
    while (!IsEndNode(current_node_index)) {
        path_station.push_back(current_node_index);
        path_start_time.push_back(solution_collector->Value(0, start_time_[current_node_index]));
        path_transfer.push_back(solution_collector->Value(0, transfer_[current_node_index]));
        current_node_index = next_values[current_node_index];
    }

    std::vector<int64> path_duration;
    const auto node_size = path_station.size();
    for (auto node_index = 1; node_index < node_size; ++node_index) {
        path_duration.push_back(path_start_time[node_index] - path_start_time[node_index - 1]);
    }
    path_duration.push_back(end_stage_index_ - path_start_time[node_size - 1]);

    std::vector<CpSolution::Job> jobs;
    for (auto node_index = 0; node_index < node_size; ++node_index) {
        jobs.emplace_back(path_start_time[node_index],
                          path_duration[node_index],
                          path_station[node_index],
                          path_transfer[node_index]);
    }

    return {std::move(jobs)};
}

void quake::StagePathFormulation::CheckConsistency(int64 solution_number,
                                                   operations_research::SolutionCollector const *solution_collector) const {
    const auto stage_duration = end_stage_index_ - begin_stage_index_;
    CHECK_GT(stage_duration, 0);

    for (auto station_index = 0; station_index < stations_; ++station_index) {
        CHECK_GE(solution_collector->Value(0, transfer_[station_index]), 0);
        CHECK_LE(solution_collector->Value(0, start_transfer_time_[station_index]),
                 solution_collector->Value(0, end_transfer_time_[station_index]));
        CHECK_LE(solution_collector->Value(0, end_transfer_time_[station_index])
                 - solution_collector->Value(0, start_transfer_time_[station_index]),
                 stage_duration);
    }

    CHECK_EQ(model_->StationCount(), stations_);
    for (auto station_index = 0; station_index < stations_; ++station_index) {
        CHECK_GE(solution_collector->Value(0, transfer_[station_index]), 0);
    }
}

void quake::StagePathFormulation::Clear() {
    next_.clear();
    active_.clear();
    start_time_.clear();

    start_transfer_time_.clear();
    end_transfer_time_.clear();
    transfer_.clear();
}
