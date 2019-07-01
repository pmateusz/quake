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

#include "fixed_discretisation_scheme.h"

quake::FixedDiscretisationScheme::FixedDiscretisationScheme(const quake::InferredModel &model,
                                                            boost::posix_time::time_duration time_step)
        : model_{model},
          num_time_{static_cast<std::size_t>(model_.TimeRange())},
          dummy_station_{0},
          time_step_{std::move(time_step)} {}

quake::DiscretisationScheme::DiscretisationScheme(std::vector<quake::BaseInterval> transfer_intervals,
                                                  std::vector<quake::BaseInterval> switch_intervals)
        : TransferIntervals{std::move(transfer_intervals)},
          SwitchIntervals{std::move(switch_intervals)} {}

quake::DiscretisationScheme quake::FixedDiscretisationScheme::Build() {
    std::vector<BaseInterval> prototype_transfer_intervals;
    auto current_begin_index = 0;
    while (current_begin_index < num_time_) {
        const auto prototype = GetTransferInterval(current_begin_index);
        CHECK_LT(prototype.Begin, prototype.End);

        prototype_transfer_intervals.push_back(prototype);
        current_begin_index = prototype.End;
    }

    std::vector<BaseInterval> prototype_switch_intervals;
    for (const auto &interval :prototype_transfer_intervals) {
        prototype_switch_intervals.push_back(GetSwitchInterval(interval.Begin));
    }

    for (auto prototype_index = 0; prototype_index < prototype_transfer_intervals.size(); ++prototype_index) {
        CHECK_EQ(prototype_switch_intervals[prototype_index].End,
                 prototype_transfer_intervals[prototype_index].Begin);
    }

    return {std::move(prototype_transfer_intervals), std::move(prototype_switch_intervals)};
}

quake::BaseInterval quake::FixedDiscretisationScheme::GetTransferInterval(std::size_t begin_index) const {
    const auto &time_offset = model_.TimeOffset();
    const auto duration_from_start = model_.Time(begin_index) - model_.StartTime();
    const auto remainder = duration_from_start.total_seconds() % time_step_.total_seconds();

    // if interval is not aligned build interval smaller than the time step otherwise return standard interval
    if (remainder == 0) {
        // interval is aligned
        // --> return an interval as large as the time step

        const auto end_index = std::min(num_time_, begin_index + time_step_.total_seconds());
        auto last_interval_index = end_index - 1;
        while ((last_interval_index - begin_index <
                time_offset[last_interval_index] - time_offset[begin_index])
               && (last_interval_index > begin_index)) {
            --last_interval_index;
        }

        const auto end_index_to_use = last_interval_index + 1;
        CHECK_LE(end_index_to_use, end_index);

        return {dummy_station_, begin_index, end_index_to_use};
    } else {
        // interval is not aligned
        // --> return smaller interval so the following interval is aligned
        const auto max_interval_length = time_step_.total_seconds() - remainder;
        CHECK_GT(max_interval_length, 0);

        auto end_index = begin_index + 1;
        for (; end_index < num_time_ && (end_index - begin_index) < max_interval_length; ++end_index);

        auto last_interval_index = end_index - 1;
        while ((last_interval_index - begin_index <
                time_offset[last_interval_index] - time_offset[begin_index])
               && (last_interval_index > begin_index)) {
            --last_interval_index;
        }

        const auto end_index_to_use = last_interval_index + 1;
        CHECK_LE(end_index_to_use, end_index);

        return {dummy_station_, begin_index, end_index_to_use};
    }
}

quake::BaseInterval quake::FixedDiscretisationScheme::GetSwitchInterval(std::size_t transfer_start_index) const {
    const auto &time_offset = model_.TimeOffset();
    const auto switch_duration = model_.SwitchDuration();

    if (transfer_start_index == 0) {
        return {dummy_station_, 0, 0};
    }

    const auto end_index = transfer_start_index;
    const auto last_interval_index = end_index - 1;
    CHECK_GE(last_interval_index, 0);

    if (time_offset[end_index] - time_offset[last_interval_index] > switch_duration) {
        return {dummy_station_, transfer_start_index, transfer_start_index};
    }

    auto begin_index = std::max(static_cast<int64>(0), static_cast<int64>(end_index) - switch_duration);
    while (time_offset[last_interval_index] - time_offset[begin_index] > switch_duration
           && begin_index < last_interval_index) { ++begin_index; }

    if (time_offset[last_interval_index] - time_offset[begin_index] > switch_duration) {
        return {dummy_station_, last_interval_index, end_index};
    }
    return {dummy_station_, static_cast<std::size_t>(begin_index), end_index};
}
