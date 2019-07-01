//
// Copyright 2019 Mateusz Polnik
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

#ifndef QUAKE_BENDERS_CALLBACK_H
#define QUAKE_BENDERS_CALLBACK_H

#include <gurobi_c++.h>

#include "base_interval_mip_model.h"

namespace quake {

    class BendersCallback : public GRBCallback {
    public:
        BendersCallback(BaseIntervalMipModel &model, double target_traffic_index)
                : model_{model},
                  target_traffic_index_{target_traffic_index} {}

    protected:
        double DistanceToTarget(std::size_t scenario_index, std::size_t station_index) {
            std::size_t keys_received = 0;
            for (auto &interval : model_.StationIntervals(station_index)) {
                const auto interval_status = getSolution(interval.Var);
                if (util::IsActive(interval_status)) {
                    keys_received += model_.KeysTransferred(scenario_index, interval);
                }
            }

            const auto local_index = keys_received / model_.TransferShare(station_index);
            return target_traffic_index_ - local_index;
        }

    protected:
        BaseIntervalMipModel &model_;
        double target_traffic_index_;
    };
}


#endif //QUAKE_BENDERS_CALLBACK_H
