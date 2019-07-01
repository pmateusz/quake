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

#include <limits>

#include "worst_case_mip_model.h"
#include "fixed_discretisation_scheme.h"

quake::WorstCaseMipModel::WorstCaseMipModel(quake::InferredModel const *model,
                                            boost::posix_time::time_duration time_step,
                                            std::vector<quake::Forecast> forecasts)
        : BaseIntervalMipModel(model, std::move(time_step), std::move(forecasts)) {}

void quake::WorstCaseMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // variable: traffic index
    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    auto traffic_index = mip_model_.addVar(0.0, traffic_index_ub, 0.0, GRB_CONTINUOUS, "traffic_index");

    // constraint: for each station and each scenario traffic index is satisfied
    const auto num_scenarios = scenario_pool_.size();
    for (auto station_index = 1; station_index < num_stations_; ++station_index) {
        const auto num_intervals = intervals_.at(station_index).size();
        for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
            GRBLinExpr keys_transferred = 0;
            for (auto interval_index = 0; interval_index < num_intervals; ++interval_index) {
                auto &interval = intervals_.at(station_index).at(interval_index);
                keys_transferred += scenario_pool_.KeysTransferred(scenario_index, interval) * interval.Var;
            }
            mip_model_.addConstr(model_->TransferShare(station_index) * traffic_index <= keys_transferred);
        }
    }

    // objective:
    GRBLinExpr objective = traffic_index;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    mip_model_.setObjective(objective);
}

double quake::WorstCaseMipModel::GetTrafficIndex(const quake::Solution &solution) const {
    double global_traffic_index = std::numeric_limits<double>::max();

    for(const auto &forecast : forecasts_) {
        for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
            int64 keys_transferred = 0;
            const auto station = model_->Station(station_index);
            for (const auto &observation_window : solution.ObservationWindows(station)) {
                keys_transferred += model_->WeatherAdjustedTransferredKeys(station,
                                                                           observation_window.begin(),
                                                                           observation_window.end(),
                                                                           forecast);
            }
            double local_traffic_index = keys_transferred / model_->TransferShare(station_index);
            global_traffic_index = std::min(global_traffic_index, local_traffic_index);
        }
    }

    return global_traffic_index;
}
