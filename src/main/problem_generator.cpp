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

#include "problem_generator.h"

#include <unordered_map>
#include <numeric>
#include <tuple>

#include <glog/logging.h>

#include "util/resources.h"
#include "util/hash.h"

#include "transfer_rate_reader.h"
#include "kepler_elements.h"
#include "elevation.h"
#include "sunset_sunrise_reader.h"
#include "problem.h"
#include "key_consumption_engine.h"

static bool inline can_transfer(double elevation_angle) {
    return elevation_angle >= quake::util::MIN_ELEVATION;
}

static const auto DEFAULT_SWITCH_DURATION = boost::posix_time::seconds(30);

static const int DEFAULT_TRANSFER_RATE = 633;

void OutputWarningIfDistributionCoefficientIsSmall(
        const std::unordered_map<quake::GroundStation, double> &cumulative_keys) {
    for (const auto &record : cumulative_keys) {
        LOG_IF(WARNING, record.second < 0.01) << "Distribution coefficient for " << record.first.name()
                                              << " is very small: " << record.second;
    }
}

std::unordered_map<quake::GroundStation, double>
quake::ProblemGenerator::GetLinearDistributionCoefficients(const std::vector<quake::GroundStation> &ground_stations) {
    static const std::unordered_map<quake::GroundStation, double> Weights = {
            {quake::GroundStation::London,     0.25},
            {quake::GroundStation::Thurso,     0.01},
            {quake::GroundStation::Cambridge,  0.05},
            {quake::GroundStation::Birmingham, 0.15},
            {quake::GroundStation::Glasgow,    0.12},
            {quake::GroundStation::Manchester, 0.12},
            {quake::GroundStation::Bristol,    0.10},
            {quake::GroundStation::Ipswich,    0.15},
            {quake::GroundStation::York,       0.05}
    };

    double total_weight = 0.0;
    std::unordered_map<quake::GroundStation, double> stations_considered;
    for (const auto &station :ground_stations) {
        const auto station_it = Weights.find(station);
        CHECK(station_it != std::cend(Weights));

        total_weight += station_it->second;
        stations_considered.emplace(station_it->first, station_it->second);
    }

    CHECK_GT(total_weight, 0.0);
    CHECK_LE(total_weight, 1.0);

    if (total_weight == 1.0) {
        return stations_considered;
    }

    double increase_by = 1.0 / total_weight;
    LOG_IF(WARNING, increase_by > 100.0) << "Weights of ground stations are increased by factor of " << increase_by;

    double total_weight_reexamined = 0.0;
    for (auto &station_record : stations_considered) {
        station_record.second *= increase_by;
        total_weight_reexamined += station_record.second;
    }
    CHECK_NEAR(total_weight_reexamined, 1.0, 0.001);

    return stations_considered;
}

std::unordered_map<quake::GroundStation, double> GetDistributionCoefficientsFromTransferMatrix(
        const std::vector<quake::GroundStation> &ground_stations) {
    static const std::vector<std::tuple<quake::GroundStation, quake::GroundStation, int64> > TransferRequests{
            std::make_tuple(quake::GroundStation::London, quake::GroundStation::Manchester, 2),
            std::make_tuple(quake::GroundStation::Manchester, quake::GroundStation::Glasgow, 3),
            std::make_tuple(quake::GroundStation::London, quake::GroundStation::Glasgow, 4)
    };

    std::unordered_map<quake::GroundStation, int64> cumulative_keys;
    for (const auto &ground_station : ground_stations) {
        cumulative_keys.emplace(ground_station, 0);
    }

    for (const auto &request : TransferRequests) {
        const auto left_station = std::get<0>(request);
        auto left_it = cumulative_keys.find(left_station);
        if (left_it == std::cend(cumulative_keys)) {
            continue;
        }

        const auto right_station = std::get<1>(request);
        auto right_it = cumulative_keys.find(right_station);
        if (right_it == std::cend(cumulative_keys)) {
            continue;
        }

        const auto keys = std::get<2>(request);
        left_it->second += keys;
        right_it->second += keys;
    }

    auto total_keys = 0;
    for (const auto &record :cumulative_keys) {
        total_keys += record.second;
    }

    std::unordered_map<quake::GroundStation, double> distribution_coefficients;
    for (const auto &cumulative_record : cumulative_keys) {
        const auto coefficient = static_cast<double>(cumulative_record.second) / total_keys;
        distribution_coefficients.emplace(cumulative_record.first, coefficient);
    }

    OutputWarningIfDistributionCoefficientIsSmall(distribution_coefficients);
    return distribution_coefficients;
}

std::unordered_map<quake::GroundStation, double> GetDistributionCoefficientsFromPopulation(
        const std::vector<quake::GroundStation> &ground_stations) {
    static const std::unordered_map<quake::GroundStation, int64> Population = {
            {quake::GroundStation::London,     8825000},
            {quake::GroundStation::Thurso,     7933},
            {quake::GroundStation::Cambridge,  124900},
            {quake::GroundStation::Birmingham, 1137100},
            {quake::GroundStation::Glasgow,    621020},
            {quake::GroundStation::Manchester, 545500},
            {quake::GroundStation::Bristol,    459300},
            {quake::GroundStation::Ipswich,    133384},
            {quake::GroundStation::York,       208200}
    };

    std::vector<quake::GroundStation> station;
    std::vector<int64> capacity;
    for (const auto &ground_station : ground_stations) {
        station.push_back(ground_station);
        capacity.push_back(Population.at(ground_station));
    }
    quake::MaxFlowEngine flow_engine;
    const auto flow_matrix = flow_engine.ComputeFlow(capacity);

    const auto station_size = station.size();
    std::vector<int64> total_station_flow(station_size, 0);
    for (auto station_index = 0; station_index < station_size; ++station_index) {
        total_station_flow[station_index] = std::accumulate(std::cbegin(flow_matrix[station_index]),
                                                            std::cend(flow_matrix[station_index]),
                                                            static_cast<int64>(0));
    }
    int64 total_flow = std::accumulate(std::cbegin(total_station_flow),
                                       std::cend(total_station_flow),
                                       static_cast<int64>(0));

    std::unordered_map<quake::GroundStation, double> distribution_coefficients;
    for (auto station_index = 0; station_index < station_size; ++station_index) {
        distribution_coefficients[station[station_index]]
                = static_cast<double>(total_station_flow[station_index]) / total_flow;
    }

    OutputWarningIfDistributionCoefficientIsSmall(distribution_coefficients);
    return distribution_coefficients;
}

std::unordered_map<quake::GroundStation, int> GetInitialBuffers(
        std::unordered_map<quake::GroundStation, double> distribution_coefficients,
        int days) {
    CHECK_GT(days, 0);

    std::unordered_map<quake::GroundStation, int> initial_buffers;
    for (const auto &distribution_coefficient : distribution_coefficients) {
        static const auto KEY_SCALING_FACTOR = 10000;
        // make the buffer size divisible by the number of days
        const int buffer_size = static_cast<int>((distribution_coefficient.second * KEY_SCALING_FACTOR) / days) * days;
        initial_buffers.emplace(distribution_coefficient.first, buffer_size);
    }
    return initial_buffers;
}

std::unordered_map<quake::GroundStation, int> GetDailyKeyConsumption(
        std::unordered_map<quake::GroundStation, int> initial_buffers,
        int days) {
    CHECK_GT(days, 0);

    std::unordered_map<quake::GroundStation, int> key_consumption;
    for (const auto &initial_buffer : initial_buffers) {
        const auto daily_key_consumption = initial_buffer.second / days;
        CHECK_GT(daily_key_consumption, 0);

        key_consumption.emplace(initial_buffer.first, daily_key_consumption);
    }
    return key_consumption;
}

quake::Problem quake::ProblemGenerator::Create(std::vector<quake::GroundStation> ground_stations,
                                               boost::posix_time::ptime initial_epoch,
                                               boost::posix_time::time_period time_period) const {
    static const auto TIME_STEP = boost::posix_time::seconds(1);

    std::unordered_map<quake::GroundStation, std::vector<double>> elevation_angle_data;
    for (const auto &ground_station : ground_stations) {
        auto elevation = GetElevation(
                ground_station,
                KeplerElements::DEFAULT,
                initial_epoch,
                time_period,
                TIME_STEP);
        std::transform(std::begin(elevation), std::end(elevation), std::begin(elevation), Util::RadiansToDegrees);
        elevation_angle_data.emplace(ground_station, std::move(elevation));
    }

    quake::util::Resources resources{"~/dev/quake/data"};
    quake::SunsetSunriseReader sunset_sunrise_reader{quake::util::Resources::DEFAULT_LOCAL_TIME_ZONE};
    std::unordered_map<quake::GroundStation,
            std::unordered_map<boost::gregorian::date,
                    std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > > sunset_sunrise_data;

    for (const auto &ground_station : ground_stations) {
        auto sunset_sunrise_pairs = sunset_sunrise_reader.Read(resources.SunsetSunriseData(ground_station));
        DCHECK_EQ(ground_station, sunset_sunrise_pairs.first);

        std::unordered_map<boost::gregorian::date,
                std::pair<boost::posix_time::ptime, boost::posix_time::ptime>> sunset_sunrise_row{};
        for (const auto &sunset_sunrise_pair : sunset_sunrise_pairs.second) {
            DCHECK_EQ(sunset_sunrise_pair.first.date(), sunset_sunrise_pair.second.date());

            sunset_sunrise_row.emplace(sunset_sunrise_pair.first.date(), sunset_sunrise_pair);
        }
        sunset_sunrise_data.emplace(ground_station, sunset_sunrise_row);
    }

    const auto start_time = time_period.begin();
    std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period>> transfer_window_data;
    for (const auto &ground_station : ground_stations) {
        const auto &ground_station_elevation = elevation_angle_data[ground_station];

        std::vector<boost::posix_time::time_period> elevation_windows;
        const auto elevation_size = ground_station_elevation.size();
        for (auto begin_transfer_delta = 0; begin_transfer_delta < elevation_size; ++begin_transfer_delta) {
            if (can_transfer(ground_station_elevation[begin_transfer_delta])) {
                auto end_transfer_delta = begin_transfer_delta + 1;
                for (; end_transfer_delta < elevation_size
                       && can_transfer(ground_station_elevation[end_transfer_delta]);
                       ++end_transfer_delta);

                elevation_windows.emplace_back(boost::posix_time::time_period(
                        start_time + boost::posix_time::seconds(begin_transfer_delta),
                        boost::posix_time::seconds(end_transfer_delta - begin_transfer_delta)));
                begin_transfer_delta = end_transfer_delta;
            }
        }

        std::vector<boost::posix_time::time_period> night_windows;
        for (auto current_day = start_time.date(); current_day < time_period.end().date();) {
            const auto next_day = current_day + boost::gregorian::days(1);
            const auto sunset_it = sunset_sunrise_data[ground_station].find(current_day);
            if (sunset_it == std::end(sunset_sunrise_data[ground_station])) {
                std::stringstream msg;
                msg << "Sunset not known for " << ground_station << " for day " << start_time.date();
                throw std::runtime_error(msg.str());
            }

            const auto sunrise_it = sunset_sunrise_data[ground_station].find(next_day);
            if (sunset_it == std::end(sunset_sunrise_data[ground_station])) {
                std::stringstream msg;
                msg << "Sunset not known for " << ground_station << " for day " << start_time.date();
                throw std::runtime_error(msg.str());
            }

            const auto sunset = sunset_it->second.second;
            const auto sunrise = sunrise_it->second.first;
            night_windows.emplace_back(sunset, sunrise);

            current_day = next_day;
        }

        std::vector<boost::posix_time::time_period> transfer_windows;
        auto elevation_window_it = std::cbegin(elevation_windows);
        auto night_window_it = std::cbegin(night_windows);
        const auto elevation_window_end_it = std::cend(elevation_windows);
        const auto night_window_end_it = std::cend(night_windows);
        for (; elevation_window_it != elevation_window_end_it && night_window_it != night_window_end_it;) {
            const auto elevation_window = *elevation_window_it;
            const auto night_window = *night_window_it;

            if (elevation_window.is_after(night_window.end())) {
                ++night_window_it;
                continue;
            } else if (elevation_window.is_before(night_window.begin())) {
                ++elevation_window_it;
                continue;
            } else {
                DCHECK(elevation_window.intersects(night_window) || night_window.contains(elevation_window));

                if (night_window.contains(elevation_window)) {
                    transfer_windows.push_back(elevation_window);
                } else {
                    auto intersection_window = elevation_window.intersection(night_window);
                    if (intersection_window.length() > boost::posix_time::seconds(0)) {
                        transfer_windows.emplace_back(intersection_window);
                    }
                }

                ++elevation_window_it;
            }
        }

        transfer_window_data.emplace(ground_station, std::move(transfer_windows));
    }

    const auto length_days = time_period.length().hours() / 24;
    const auto distribution_coefficients = GetLinearDistributionCoefficients(ground_stations);
    const auto initial_buffers = GetInitialBuffers(distribution_coefficients, length_days);
    const auto key_consumption = GetDailyKeyConsumption(initial_buffers, length_days);

    if (VLOG_IS_ON(1)) {
        std::ostringstream output;

        output << "Generated problem: " << std::endl;
        for (const auto &station_data : transfer_window_data) {
            output << "Station: " << station_data.first << std::endl;
            output << " - Transfer windows: ";

            auto window_it = std::cbegin(station_data.second);
            const auto window_end_it = std::cend(station_data.second);
            if (window_it != window_end_it) {
                output << *window_it;
                ++window_it;
            }

            for (; window_it != window_end_it; ++window_it) {
                output << " ";
                output << *window_it;
            }
            output << std::endl;
            output << " - Distribution coefficient: " << distribution_coefficients.at(station_data.first) << std::endl;
        }

        VLOG(1) << output.str();
    }

    quake::TransferRateReader transfer_rate_reader;
    auto transfer_rate_data = transfer_rate_reader.Read(resources.TransferRate(DEFAULT_TRANSFER_RATE));

    return {time_period,
            std::move(ground_stations),
            std::move(transfer_window_data),
            std::move(elevation_angle_data),
            std::move(distribution_coefficients),
            std::move(initial_buffers),
            std::move(key_consumption),
            std::move(transfer_rate_data),
            DEFAULT_SWITCH_DURATION};
}
