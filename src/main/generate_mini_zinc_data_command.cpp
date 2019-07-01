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

#include <cmath>
#include <numeric>
#include <utility>
#include <string>
#include <vector>

#include <boost/date_time.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <glog/logging.h>

#include "generate_mini_zinc_data_command.h"
#include "util/error.h"
#include "problem.h"
#include "problem_generator.h"
#include "ground_station.h"
#include "minizinc_writer.h"
#include "minizinc_data_model.h"

quake::GenerateMiniZincDataCommand::GenerateMiniZincDataCommand(boost::posix_time::time_period time_period,
                                                                boost::posix_time::time_duration time_step,
                                                                boost::filesystem::path output,
                                                                std::vector<GroundStation> ground_stations,
                                                                bool convert_float_to_int)
        : time_period_(time_period),
          time_step_(std::move(time_step)),
          output_(std::move(output)),
          ground_stations_(std::move(ground_stations)),
          convert_float_to_int_(convert_float_to_int) {}

template<typename ValueType>
static void copy_segments(const std::vector<ValueType> &source,
                          std::vector<ValueType> &destination,
                          const std::vector<std::pair<int, int> > &segments) {
    auto current_source_pos = 0;
    auto segment_it = std::cbegin(segments);
    auto segment_it_end = std::cend(segments);
    auto destination_it = std::begin(destination);
    auto destination_it_end = std::end(destination);

    for (; segment_it != segment_it_end; ++segment_it) {
        for (; current_source_pos < segment_it->first &&
               destination_it != destination_it_end;
               ++current_source_pos, ++destination_it) {
            *destination_it = source[current_source_pos];
        }
        current_source_pos += segment_it->second;
    }

    const auto source_size = source.size();
    for (; current_source_pos < source_size; ++current_source_pos, ++destination_it) {
        *destination_it = source[current_source_pos];
    }
}

static std::pair<boost::posix_time::ptime, boost::posix_time::ptime> get_start_end_mini_zinc_time(
        const quake::Problem &problem) {
    boost::posix_time::ptime min_start_time = boost::posix_time::max_date_time;
    boost::posix_time::ptime max_end_time = boost::posix_time::min_date_time;
    for (const auto &ground_station : problem.GroundStations()) {
        for (const auto &transfer_window : problem.TransferWindows(ground_station)) {
            min_start_time = std::min(min_start_time, transfer_window.begin());
            max_end_time = std::max(max_end_time, transfer_window.end());
        }
    }

    CHECK_NE(min_start_time, boost::posix_time::max_date_time);
    CHECK_NE(max_end_time, boost::posix_time::min_date_time);

    const boost::posix_time::ptime start_time{min_start_time.date(), {min_start_time.time_of_day().hours(), 0, 0}};
    const auto end_time = start_time + boost::posix_time::hours(
            ((max_end_time - start_time).total_seconds() / boost::posix_time::hours(1).total_seconds() + 1));

    return {start_time, end_time};
}

template<typename Array2DType, typename WriterType>
void OutputArray(const Array2DType &key_rate, const Array2DType &cumul_key_rate, WriterType &writer) {
    writer.WriteArray2d("key_rate",
                        std::cbegin(key_rate),
                        std::cend(key_rate));
    writer.WriteNewLine();
    writer.WriteArray2d("key_rate_cumul",
                        std::cbegin(cumul_key_rate),
                        std::cend(cumul_key_rate));
}

void quake::GenerateMiniZincDataCommand::Run() {
    const quake::ProblemGenerator generator;

    const boost::posix_time::ptime initial_epoch(boost::gregorian::date(time_period_.begin().date().year(), 1, 1),
                                                 boost::posix_time::seconds(0));

    const auto problem = generator.Create(ground_stations_, initial_epoch, time_period_);
    const auto start_end_time_pair = get_start_end_mini_zinc_time(problem);
    const auto start_time = start_end_time_pair.first;
    const auto end_time = start_end_time_pair.second;
    const std::size_t slot_count =
            static_cast<std::size_t>(
                    ceil(static_cast<double>((end_time - start_time).total_seconds()) / time_step_.total_seconds())
            );

    std::vector<int> time_delta_dim(slot_count);
    auto slot_start = start_time;
    for (int slot_index = 0; slot_index < slot_count; ++slot_index) {
        time_delta_dim[slot_index] = (slot_start - start_time).total_seconds();
        slot_start += time_step_;
    }

    std::vector<std::vector<double>> key_rates;
    key_rates.emplace_back(std::vector<double>(slot_count, 0.0));

    for (const auto &ground_station : problem.GroundStations()) {
        std::vector<double> local_key_rates(slot_count);
        auto time_it = std::cbegin(time_delta_dim);
        const auto time_it_end = std::cend(time_delta_dim);
        const auto time_step_seconds = time_step_.total_seconds();
        for (auto slot_index = 0; slot_index < slot_count && time_it != time_it_end; ++slot_index, ++time_it) {
            boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean> > transfer_rate;
            for (int current_time_delta = *time_it, slot_end = *time_it + time_step_seconds;
                 current_time_delta < slot_end;
                 ++current_time_delta) {
                transfer_rate(problem.TransferKeyRate(ground_station,
                                                      start_time +
                                                      boost::posix_time::seconds(current_time_delta)));
            }
            local_key_rates[slot_index] = boost::accumulators::mean(transfer_rate) * time_step_seconds;
        }
        DCHECK(time_it == time_it_end);

        key_rates.emplace_back(std::move(local_key_rates));
    }

    std::vector<std::pair<int, int> > zero_segments;
    const auto row_count = key_rates.size();
    for (auto current_slot = 0; current_slot < slot_count;) {
        auto start_zero_slot = current_slot;
        auto zero_series_length = 0;

        int current_row;
        while (current_slot < slot_count) {
            current_row = 0;
            for (; current_row < row_count && key_rates[current_row][current_slot] == 0.0; ++current_row);
            ++current_slot;

            if (current_row == row_count) {
                ++zero_series_length;
            } else {
                break;
            }
        }

        if (zero_series_length > 0) {
            zero_segments.emplace_back(start_zero_slot, zero_series_length);
        }
    }

    auto truncated_slot_count = slot_count;
    for (const auto &start_length : zero_segments) {
        truncated_slot_count -= start_length.second;
    }
    DCHECK_GE(truncated_slot_count, 0);

    std::vector<int> truncated_time_delta_dim(truncated_slot_count);
    copy_segments(time_delta_dim, truncated_time_delta_dim, zero_segments);

    std::vector<std::vector<double> > truncated_key_rate(row_count, std::vector<double>(truncated_slot_count));
    for (auto current_row = 0; current_row < row_count; ++current_row) {
        copy_segments(key_rates[current_row], truncated_key_rate[current_row], zero_segments);
    }

    // first segment in cumul is always zero
    const auto truncated_cumul_slot_count = truncated_slot_count + 1;

    std::ofstream output_stream;
    output_stream.open(output_.string(), std::ofstream::trunc);
    if (!output_stream) {
        throw util::OnFailedSaveOutput(output_);
    }

    MiniZincWriter<std::ofstream> writer(std::move(output_stream));
    writer.WriteVariable(MiniZincData<int>::START_TIME_REF, writer.Quote(start_time));
    writer.WriteVariable(MiniZincData<int>::END_TIME_REF, writer.Quote(end_time));
    writer.WriteVariable(MiniZincData<int>::SWITCH_DURATION_REF, problem.SwitchDuration().total_seconds());

    std::vector<std::string> station_names{quake::GroundStation::None.name()};
    std::vector<double> transfer_share{0.0};
    std::vector<int> initial_buffer{0};
    std::vector<int> key_consumption{0};
    for (const auto &station : problem.GroundStations()) {
        station_names.emplace_back(station.name());
        transfer_share.push_back(problem.TransferShare(station));
        initial_buffer.push_back(problem.InitialBuffer(station));
        key_consumption.push_back(problem.KeyConsumption(station));
    }

    writer.WriteEnum(MiniZincData<int>::STATION_REF, std::cbegin(station_names), std::cend(station_names));
    writer.WriteVariable(MiniZincData<int>::DUMMY_STATION_REF, "None");
    writer.WriteArray1d(MiniZincData<int>::TRANSFER_SHARE_REF, std::cbegin(transfer_share), std::cend(transfer_share));
    writer.WriteArray1d(MiniZincData<int>::INITIAL_BUFFER_REF,
                        std::cbegin(initial_buffer),
                        std::cend(initial_buffer));
    writer.WriteArray1d(MiniZincData<int>::KEY_CONSUMPTION_REF,
                        std::cbegin(key_consumption),
                        std::cend(key_consumption));
    writer.WriteRange(MiniZincData<int>::TIME_REF, 1, truncated_slot_count);
    writer.WriteVariable(MiniZincData<int>::STEP_DURATION_REF, time_step_.total_seconds());
    writer.WriteArray1d(MiniZincData<int>::TIME_OFFSET_REF,
                        std::cbegin(truncated_time_delta_dim),
                        std::cend(truncated_time_delta_dim));
    writer.WriteNewLine();

    if (convert_float_to_int_) {
        std::vector<std::vector<int64_t> > casted_truncated_key_rate;
        for (const auto &row : truncated_key_rate) {
            std::vector<int64_t> casted_row;

            for (const auto &value : row) {
                casted_row.push_back(static_cast<int64_t >(round(value)));
            }

            casted_truncated_key_rate.emplace_back(std::move(casted_row));
        }

        std::vector<std::vector<int64_t> > casted_truncated_cumul_key_rate(row_count,
                                                                           std::vector<int64_t>(
                                                                                   truncated_cumul_slot_count));
        if (truncated_slot_count > 0) {
            for (auto current_row = 0; current_row < row_count; ++current_row) {
                const auto &row = casted_truncated_key_rate[current_row];
                auto &row_cumul = casted_truncated_cumul_key_rate[current_row];
                row_cumul[0] = 0;
                for (auto current_slot = 1; current_slot < truncated_cumul_slot_count; ++current_slot) {
                    row_cumul[current_slot] = row_cumul[current_slot - 1] + row[current_slot - 1];
                }
            }
        }

#if DEBUG
        // ENSURE that last cumul is equal to sum of all rows
        for (auto row = 0; row < row_count; ++row) {
            auto total_key_rate = std::accumulate(std::cbegin(casted_truncated_key_rate[row]),
                                                  std::cend(casted_truncated_key_rate[row]),
                                                  static_cast<int64_t>(0));
            CHECK_EQ(total_key_rate, casted_truncated_cumul_key_rate[row].at(truncated_cumul_slot_count - 1));
        }
#endif

        OutputArray(casted_truncated_key_rate, casted_truncated_cumul_key_rate, writer);
    } else {
        std::vector<std::vector<double> > truncated_key_rate_cumul(row_count,
                                                                   std::vector<double>(truncated_cumul_slot_count));
        if (truncated_slot_count > 0) {
            for (auto current_row = 0; current_row < row_count; ++current_row) {
                const auto &row = truncated_key_rate[current_row];
                auto &row_cumul = truncated_key_rate_cumul[current_row];
                row_cumul[0] = 0;
                for (auto current_slot = 1; current_slot < truncated_cumul_slot_count; ++current_slot) {
                    row_cumul[current_slot] = row_cumul[current_slot - 1] + row[current_slot - 1];
                }
            }
        }

#if DEBUG
        // ENSURE that last cumul is equal to sum of all rows
        for (auto row = 0; row < row_count; ++row) {
            auto total_key_rate = std::accumulate(std::cbegin(truncated_key_rate[row]),
                                                  std::cend(truncated_key_rate[row]),
                                                  0.0);
            CHECK_NEAR(total_key_rate, truncated_key_rate_cumul[row].at(truncated_cumul_slot_count - 1), 0.000001);
        }
#endif

        OutputArray(truncated_key_rate, truncated_key_rate_cumul, writer);
    }

    writer.Close();
}
