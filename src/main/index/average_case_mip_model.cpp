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

quake::AverageCaseMipModel::AverageCaseMipModel(ExtendedProblem const *problem,
                                                boost::posix_time::time_duration interval_step,
                                                std::vector<Forecast> forecasts)
        : BaseIntervalMipModel(problem, std::move(interval_step), std::move(forecasts)) {}


void quake::AverageCaseMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // variable: traffic index
    const auto traffic_index_upper_bound = GetTrafficIndexUpperBound();
    auto traffic_index = mip_model_.addVar(0.0, traffic_index_upper_bound, 0.0, GRB_CONTINUOUS, "traffic_index");

    // constraints: for each station restrict the traffic index
    for (const auto &station :Stations()) {
        if (station == GroundStation::None) { continue; }

        GRBLinExpr keys_transferred;
        for (const auto &interval : StationIntervals(station)) {
            keys_transferred += KeyRate(station, interval.Period()) * interval.Var();
        }
        mip_model_.addConstr(TransferShare(station) * traffic_index <= keys_transferred);
    }

    // objective:
    GRBLinExpr objective = traffic_index;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    mip_model_.setObjective(objective);
}
