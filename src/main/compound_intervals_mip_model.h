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

#ifndef QUAKE_COMPOUND_INTERVALS_MIP_MODEL_H
#define QUAKE_COMPOUND_INTERVALS_MIP_MODEL_H

#include <vector>
#include <memory>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include <gurobi_c++.h>

#include <glog/logging.h>

#include "base_interval.h"
#include "base_mip_model.h"
#include "index/scenario_pool.h"

namespace quake {

    class CompoundIntervalsMipModel : public BaseMipModel {
    public:
        CompoundIntervalsMipModel(quake::InferredModel const *model,
                                  Forecast forecast,
                                  boost::posix_time::time_duration time_step)
                : BaseMipModel(model, std::move(time_step)),
                  forecast_{std::move(forecast)},
                  scenario_pool_{} {}

    protected:
        void Build(const boost::optional<Solution> &solution) override {
            CHECK(intervals_.empty());

            const auto intervals = ComputeIntervals();

            // for each ground
            std::vector<int64> max_lambdas;
            const auto last_time_slot = model_->TimeRange();
            for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
                const auto index = model_->CumulativeIndex(station_index, last_time_slot);
                const auto best_station_lambda = static_cast<int64>(ceil(
                        static_cast<double>(model_->KeyRateCumulative1d().at(index))
                        / model_->TransferShare(station_index)));
                max_lambdas.push_back(best_station_lambda);
            }

            // create variables for intervals
            for (const auto &prototype : intervals) {
                intervals_.emplace_back(VarInterval::Create(mip_model_, prototype));
            }

            // constraint: at most one interval is active
            std::vector<GRBLinExpr> active_intervals(num_time_);
            for (const auto &interval : intervals_) {
                const std::size_t end_to_use = std::min(num_time_ - 1, interval.End);
                for (auto time_index = interval.Begin; time_index <= end_to_use; ++time_index) {
                    active_intervals.at(time_index) += interval.Var;
                }
            }
            for (auto time_index = 0; time_index < num_time_; ++time_index) {
                mip_model_.addConstr(active_intervals.at(time_index) <= 1.0);
            }

            // constraint: lambda is bounded from above
            const double max_lambda = *std::max_element(std::cbegin(max_lambdas), std::cend(max_lambdas));
            const double sum_max_lambda = std::accumulate(std::cbegin(max_lambdas), std::cend(max_lambdas), 0.0);
            GRBVar lambda = mip_model_.addVar(0.0, max_lambda, sum_max_lambda, GRB_CONTINUOUS, "lambda");

            std::vector<GRBLinExpr> keys_delivered(num_stations_);
            for (const auto &interval : intervals_) {
                if (interval.StationIndex != dummy_station_) {
                    keys_delivered.at(interval.StationIndex)
                            += scenario_pool_.KeysTransferred(0, interval) * interval.Var;
                }
            }

            for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
                mip_model_.addConstr(model_->TransferShare(station_index) * lambda <= keys_delivered.at(station_index));
            }

            // objective function
            mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
        }

        std::unordered_map<GroundStation,
                std::vector<boost::posix_time::time_period> > GetObservations() const override {
            std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period> > observations;
            for (const auto &interval : intervals_) {
                if (util::IsActive(interval.Var) && interval.StationIndex != dummy_station_) {
                    const auto station = model_->Station(interval.StationIndex);
                    const auto start_time = model_->Time(
                            model_->GetStartTransferTimeIndex(interval.StationIndex, interval.Begin));
                    const auto end_time = model_->Time(interval.End);
                    boost::posix_time::time_period period(start_time, end_time);

                    auto find_it = observations.find(station);
                    if (find_it == std::end(observations)) {
                        observations.emplace(station, std::vector<boost::posix_time::time_period>{period});
                    } else {
                        find_it->second.emplace_back(period);
                    }
                }
            }
            return observations;
        }

    private:
        std::vector<BaseInterval> ComputeIntervals() const {
            const auto &key_rate_array = model_->KeyRate1d();
            const auto &time_offset = model_->TimeOffset();

            const auto num_stations = model_->StationCount();
            const auto num_time = time_offset.size();
            const auto first_regular_station = 1;

            std::vector<BaseInterval> intervals;
            for (auto station_index = first_regular_station; station_index < num_stations; ++station_index) {
                const auto weather_callback = model_->GetWeatherAdjustedCumulativeKeyRate(station_index);

                for (auto begin_index = 0; begin_index < num_time; begin_index += time_step_.total_seconds()) {
                    for (auto end_index = begin_index; end_index < num_time; end_index += time_step_.total_seconds()) {
                        if (end_index - begin_index < time_offset[end_index] - time_offset[begin_index]) {
                            auto end_index_to_use = end_index;

                            CHECK_GT(end_index_to_use, begin_index);
                            while ((end_index_to_use - begin_index <
                                    time_offset[end_index_to_use] - time_offset[begin_index])
                                   && (end_index_to_use > begin_index)) {
                                --end_index_to_use;
                            }

                            CHECK_GE(end_index_to_use, begin_index);
                            if (end_index_to_use - begin_index > model_->ReconfigurationTime().at(begin_index)) {
                                const auto start_transfer_time =
                                        begin_index + model_->ReconfigurationTime().at(begin_index);
                                const auto end_transfer_time = end_index_to_use + 1;
                                const auto keys_transferred =
                                        weather_callback(end_transfer_time) - weather_callback(start_transfer_time);
                                if (keys_transferred > 0) {
                                    intervals.emplace_back(station_index, begin_index, end_index_to_use);
                                }
                            }
                            break;
                        }

                        if (end_index - begin_index > model_->ReconfigurationTime().at(begin_index)) {
                            const auto start_transfer_time =
                                    begin_index + model_->ReconfigurationTime().at(begin_index);
                            const auto end_transfer_time = end_index + 1;
                            const auto keys_transferred =
                                    weather_callback(end_transfer_time) - weather_callback(start_transfer_time);
                            if (keys_transferred > 0) {
                                intervals.emplace_back(station_index, begin_index, end_index);
                            }
                        }
                    }
                }
            }

            return intervals;
        }

        Forecast forecast_;
        ScenarioPool scenario_pool_;
        std::vector<VarInterval> intervals_;
    };
}


#endif //QUAKE_COMPOUND_INTERVALS_MIP_MODEL_H
