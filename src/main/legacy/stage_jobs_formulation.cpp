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

#include <glog/logging.h>

#include "stage_jobs_formulation.h"

quake::StageJobsFormulation::StageJobsFormulation(InferredModel const *model,
                                                  std::size_t begin_stage_index,
                                                  std::size_t end_stage_index,
                                                  int64 min_jobs,
                                                  int64 max_jobs)
        : model_{model},
          begin_stage_index_{begin_stage_index},
          end_stage_index_{end_stage_index},
          min_jobs_{min_jobs},
          max_jobs_{max_jobs} {
    CHECK_GE(min_jobs_, 0);
    CHECK_GE(max_jobs_, 0);
    CHECK_LE(min_jobs_, max_jobs_);
    CHECK_EQ(model_->DummyStationIndex(), 0);
}

void quake::StageJobsFormulation::Build(operations_research::Solver *solver) {
    // --> define stations
    // first N-1 stations must be regular stations
    for (auto job_index = 0; job_index < min_jobs_; ++job_index) {
        station_.push_back(solver->MakeIntVar(1, model_->StationNoDummyCount()));
    }

    if (max_jobs_ > min_jobs_) {
        // a dummy station may be selected for remaining stations
        for (auto job_index = min_jobs_; job_index < max_jobs_; ++job_index) {
            station_.push_back(solver->MakeIntVar(0, model_->StationNoDummyCount()));
        }
    }
    CHECK_EQ(station_.size(), max_jobs_);

    for (auto job_index = 0; job_index < max_jobs_; ++job_index) {
        active_.push_back(solver->MakeIsDifferentCstVar(station_[job_index], model_->DummyStationIndex()));
    }
    CHECK_EQ(active_.size(), max_jobs_);

    if (max_jobs_ > min_jobs_) {
        if (max_jobs_ == model_->StationNoDummyCount()) {
            // special case - use all different except 0 constraint

            solver->AddConstraint(solver->MakeAllDifferent(station_));
            solver->AddConstraint(solver->MakeAllDifferentExcept(station_, model_->DummyStationIndex()));
            solver->AddConstraint(solver->MakeAtMost(station_, model_->DummyStationIndex(), max_jobs_ - min_jobs_));
        } else {
            // special case - use distribute constraint

            // no two regular stations next to each other are the same
            for (auto station_index = 1; station_index < min_jobs_; ++station_index) {
                solver->AddConstraint(
                        solver->MakeNonEquality(station_[station_index - 1], station_[station_index]));
            }

            // no two regular stations next to each other are the same unless they are dummy
            for (auto job_index = min_jobs_; job_index < max_jobs_; ++job_index) {
                solver->AddConstraint(
                        solver->MakeLessOrEqual(
                                active_[job_index - 1],
                                solver->MakeIsDifferentVar(station_[job_index - 1], station_[job_index])
                        ));
            }

            // at least 1 transfer is dedicated to one station
            std::vector<int64> station_indices(model_->StationCount(), 0);
            for (auto station_index = 1; station_index < model_->StationCount(); ++station_index) {
                station_indices[station_index] = station_index;
            }

            std::vector<int64> min_station_occ(model_->StationCount(), 1);
            min_station_occ[0] = 0;

            std::vector<int64> max_station_occ(static_cast<std::size_t >(model_->StationCount()),
                                               max_jobs_ - min_jobs_);
            solver->AddConstraint(
                    solver->MakeDistribute(station_, station_indices, min_station_occ, max_station_occ));
        }

        for (auto job_index = 0; job_index < min_jobs_; ++job_index) {
            solver->AddConstraint(solver->MakeGreater(station_[job_index], model_->DummyStationIndex()));
        }

        // no regular station may appear after a dummy station
        for (auto job_index = min_jobs_; job_index < max_jobs_; ++job_index) {
            solver->AddConstraint(
                    solver->MakeGreaterOrEqual(
                            solver->MakeProd(station_[job_index - 1], model_->StationNoDummyCount()),
                            station_[job_index]));
        }
    } else {
        // special case - use all different constraint
        CHECK_EQ(min_jobs_, max_jobs_);
        solver->AddConstraint(solver->MakeAllDifferent(station_));
    }

    const int64 min_time = begin_stage_index_;
    const int64 max_time = end_stage_index_;
    CHECK_LE(min_time, max_time);

    // --> define start times
    for (auto station_index = 0; station_index < max_jobs_; ++station_index) {
        start_time_.push_back(solver->MakeIntVar(min_time, max_time));
    }
    CHECK_EQ(start_time_.size(), max_jobs_);

    // --> define reconfiguration time
    for (auto station_index = 0; station_index < max_jobs_; ++station_index) {
        reconfiguration_time_.push_back(solver->MakeIntVar(0, model_->SwitchDuration()));
    }
    CHECK_EQ(reconfiguration_time_.size(), max_jobs_);

    // first start time is zero
    solver->AddConstraint(solver->MakeEquality(start_time_[0], min_time));

    // first reconfiguration time is zero
    solver->AddConstraint(solver->MakeEquality(reconfiguration_time_[0], 0));

    // start times are increasing
    for (auto station_index = 1; station_index < max_jobs_; ++station_index) {
        solver->AddConstraint(
                solver->MakeLessOrEqual(start_time_[station_index - 1], start_time_[station_index]));
    }

    // start times that belong to a regular station are offset by switch time
    for (auto station_index = 1; station_index < max_jobs_; ++station_index) {
        solver->AddConstraint(
                solver->MakeEquality(active_[station_index - 1],
                                     solver->MakeIsLessVar(
                                             solver->MakeSum(start_time_[station_index - 1],
                                                             solver->MakeProd(
                                                                     active_[station_index - 1],
                                                                     solver->MakeElement(
                                                                             model_->ReconfigurationTime(),
                                                                             start_time_[station_index - 1]))),
                                             start_time_[station_index])));
    }

    // start times of a dummy station is at the end of time range
    for (auto station_index = min_jobs_; station_index < max_jobs_; ++station_index) {
        solver->AddConstraint(
                solver->MakeEquality(active_[station_index],
                                     solver->MakeIsDifferentCstVar(start_time_[station_index], max_time)));
    }

    // reconfiguration times of a dummy station is zero
    for (auto station_index = min_jobs_; station_index < max_jobs_; ++station_index) {
        solver->AddConstraint(
                solver->MakeLessOrEqual(solver->MakeDifference(1, active_[station_index]),
                                        solver->MakeIsEqualCstVar(reconfiguration_time_[station_index], 0)));
    }

    // --> start index and end index
    for (auto job_index = 0; job_index < max_jobs_; ++job_index) {
        station_offset_.push_back(solver->MakeProd(station_[job_index], model_->StationIndexMultiplier())->Var());
    }

    job_transfer_start_index_.push_back(solver->MakeSum(station_offset_[0], start_time_[0])->Var());
    for (auto job_index = 1; job_index < max_jobs_; ++job_index) {
        job_transfer_start_index_.push_back(
                solver->MakeSum(station_offset_[job_index],
                                solver->MakeSum(start_time_[job_index],
                                                solver->MakeProd(
                                                        active_[job_index],
                                                        solver->MakeElement(model_->ReconfigurationTime(),
                                                                            start_time_[job_index]))))->Var());
    }

    const auto last_action = max_jobs_ - 1;
    for (auto job_index = 1; job_index < max_jobs_; ++job_index) {
        job_transfer_end_index_.push_back(
                solver->MakeSum(station_offset_[job_index - 1], start_time_[job_index])->Var());
    }
    job_transfer_end_index_.push_back(solver->MakeSum(station_offset_[last_action], max_time)->Var());

    // --> last start index is less than end index
    solver->AddConstraint(
            solver->MakeLessOrEqual(job_transfer_start_index_[last_action], job_transfer_end_index_[last_action]));

    // --> action transfer
    for (auto job_index = 0; job_index < max_jobs_; ++job_index) {
        job_transfer_.push_back(
                solver->MakeDifference(
                        solver->MakeElement(model_->KeyRateCumulative1d(), job_transfer_end_index_[job_index]->Var()),
                        solver->MakeElement(model_->KeyRateCumulative1d(), job_transfer_start_index_[job_index]->Var())
                )->Var());
    }

    for (auto job_index = 0; job_index < max_jobs_; ++job_index) {
        solver->AddConstraint(
                solver->MakeEquality(active_[job_index],
                                     solver->MakeIsGreaterCstVar(job_transfer_[job_index], 0)));
    }

    // --> station transfer
    for (auto station_index = 0; station_index < model_->StationCount(); ++station_index) {
        std::vector<operations_research::IntVar *> partial_transfers;
        for (auto job_index = 0; job_index < max_jobs_; ++job_index) {
            partial_transfers.push_back(
                    solver->MakeProd(
                            solver->MakeIsEqualCstVar(station_[job_index], station_index),
                            job_transfer_[job_index])->Var());
        }
        station_transfer_.push_back(solver->MakeSum(partial_transfers)->Var());
    }
    solver->AddConstraint(solver->MakeEquality(station_transfer_[0], 0));
}

std::vector<quake::CpSolution::Job> quake::StageJobsFormulation::GetJobs(int solution_number,
                                                                         operations_research::SolutionCollector const *solution_collector) const {
    std::vector<quake::CpSolution::Job> jobs;
    for (auto job_index = 1; job_index < max_jobs_; ++job_index) {
        int64 duration = solution_collector->Value(solution_number, start_time_[job_index])
                         - solution_collector->Value(solution_number, start_time_[job_index - 1]);
        CHECK_GE(duration, 0);
        jobs.emplace_back(solution_collector->Value(solution_number, start_time_[job_index - 1]),
                          duration,
                          solution_collector->Value(solution_number, station_[job_index - 1]),
                          solution_collector->Value(solution_number, job_transfer_[job_index - 1]));
    }
    const auto last_action = max_jobs_ - 1;
    const auto last_duration = EndStageIndex() - solution_collector->Value(solution_number, start_time_[last_action]);
    CHECK_GE(last_duration, 0);
    jobs.emplace_back(solution_collector->Value(solution_number, start_time_[last_action]),
                      last_duration,
                      solution_collector->Value(solution_number, station_[last_action]),
                      solution_collector->Value(solution_number, job_transfer_[last_action]));

    return jobs;
}

void quake::StageJobsFormulation::Clear() {
    station_.clear();
    active_.clear();
    start_time_.clear();
    reconfiguration_time_.clear();
    station_offset_.clear();
    job_transfer_start_index_.clear();
    job_transfer_end_index_.clear();
    job_transfer_.clear();
    station_transfer_.clear();
}

void quake::StageJobsFormulation::CheckConsistency(int64 solution_number,
                                                   operations_research::SolutionCollector const *solution_collector) const {
    const auto stage_duration = end_stage_index_ - begin_stage_index_;
    CHECK_GT(stage_duration, 0);

    const auto job_size = station_.size();
    for (auto job_index = 0; job_index < job_size; job_index++) {
        CHECK_GE(solution_collector->Value(0, job_transfer_[job_index]), 0);
        CHECK_LE(solution_collector->Value(0, job_transfer_start_index_[job_index]),
                 solution_collector->Value(0, job_transfer_end_index_[job_index]));
        CHECK_LE(solution_collector->Value(0, job_transfer_end_index_[job_index]) -
                 solution_collector->Value(0, job_transfer_start_index_[job_index]), stage_duration);
        CHECK_EQ(solution_collector->Value(0, job_transfer_start_index_[job_index]) / model_->StationIndexMultiplier(),
                 solution_collector->Value(0, station_[job_index]));
        CHECK_EQ(solution_collector->Value(0, job_transfer_end_index_[job_index]) / model_->StationIndexMultiplier(),
                 solution_collector->Value(0, station_[job_index]));
    }

    CHECK_EQ(model_->StationCount(), station_transfer_.size());
    for (auto station_index = 0; station_index < model_->StationCount(); ++station_index) {
        CHECK_GE(solution_collector->Value(0, station_transfer_[station_index]), 0);
    }
}
