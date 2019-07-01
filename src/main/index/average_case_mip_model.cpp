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

#include "average_case_mip_model.h"

quake::AverageCaseMipModel::AverageCaseMipModel(quake::InferredModel const *model,
                                                boost::posix_time::time_duration time_step,
                                                std::vector<quake::Forecast> forecasts)
        : BaseIntervalMipModel(model, std::move(time_step), std::move(forecasts)) {}


void quake::AverageCaseMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // variable: traffic index
    const auto traffic_index_upper_bound = GetTrafficIndexUpperBound();
    auto traffic_index = mip_model_.addVar(0.0, traffic_index_upper_bound, 0.0, GRB_CONTINUOUS, "traffic_index");

    // constraints: for each station restrict the traffic index
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        GRBLinExpr keys_transferred;
        for (const auto &interval : intervals_.at(station_index)) {
            keys_transferred += scenario_pool_.MeanKeysTransferred(interval) * interval.Var;
        }
        mip_model_.addConstr(model_->TransferShare(station_index) * traffic_index <= keys_transferred);
    }

    // objective:
    GRBLinExpr objective = traffic_index;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    mip_model_.setObjective(objective);
}

double quake::AverageCaseMipModel::GetTrafficIndex(const quake::Solution &solution) const {
    auto traffic_index = std::numeric_limits<double>::max();

    const auto num_forecasts = forecasts_.size();
    for (auto station_index = first_regular_station_; station_index < num_stations_; ++station_index) {
        const auto station = model_->Station(station_index);
        double average_keys_transferred = 0;
        for (const auto &window : solution.ObservationWindows(station)) {
            for (auto forecast_index = 0; forecast_index < num_forecasts; ++forecast_index) {
                average_keys_transferred += model_->WeatherAdjustedTransferredKeys(station,
                                                                                   window.begin(),
                                                                                   window.end(),
                                                                                   forecasts_.at(forecast_index));
            }
        }
        average_keys_transferred /= num_forecasts;

        const auto station_traffic_index = average_keys_transferred / model_->TransferShare(station_index);
        traffic_index = std::min(traffic_index, station_traffic_index);
    }

    return traffic_index;
}
