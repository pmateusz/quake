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

quake::WorstCaseMipModel::WorstCaseMipModel(ExtendedProblem const *problem,
                                            boost::posix_time::time_duration interval_step,
                                            std::vector<Forecast> forecasts)
        : BaseIntervalMipModel(problem, std::move(interval_step), std::move(forecasts)) {}

void quake::WorstCaseMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // variable: traffic index
    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    auto traffic_index = mip_model_.addVar(0.0, traffic_index_ub, 0.0, GRB_CONTINUOUS, "traffic_index");

    // constraint: for each station and each scenario traffic index is satisfied
    const auto num_scenarios = NumScenarios();
    for (const auto &station :Stations()) {
        const auto &station_intervals = StationIntervals(station);
        for (auto scenario_index = 0; scenario_index < num_scenarios; ++scenario_index) {
            GRBLinExpr keys_transferred = 0;
            for (const auto &interval : station_intervals) {
                keys_transferred += KeyRate(scenario_index, station, interval.Period()) * interval.Var();
            }
            mip_model_.addConstr(problem_->TransferShare(station) * traffic_index <= keys_transferred);
        }
    }

    // objective:
    GRBLinExpr objective = traffic_index;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    mip_model_.setObjective(objective);
}

double quake::WorstCaseMipModel::GetTrafficIndex(const quake::Solution &solution) const {
    double global_traffic_index = std::numeric_limits<double>::max();

    for (const auto &forecast : Forecasts()) {
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            double keys_transferred = 0;
            for (const auto &observation_window : solution.ObservationWindows(station)) {
                keys_transferred += problem_->KeyRate(station, observation_window, forecast);
            }

            double local_traffic_index = keys_transferred / TransferShare(station);
            global_traffic_index = std::min(global_traffic_index, local_traffic_index);
        }
    }

    return global_traffic_index;
}
