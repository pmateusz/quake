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

#include <pykep/third_party/libsgp4/Eci.h>
#include <pykep/third_party/libsgp4/Observer.h>
#include <pykep/third_party/libsgp4/CoordTopocentric.h>

#include "util/resources.h"
#include "util/hash.h"
#include "util/constants.h"

#include "legacy/problem.h"

#include "transfer_rate_reader.h"
#include "kepler_elements.h"
#include "elevation.h"
#include "sunset_sunrise_reader.h"
#include "key_consumption_engine.h"

static bool inline can_transfer(double elevation_angle) {
    return elevation_angle >= quake::util::MIN_ELEVATION;
}

static const auto DEFAULT_SWITCH_DURATION = boost::posix_time::seconds(30);

static const int DEFAULT_TRANSFER_RATE = 633;

void OutputWarningIfDistributionCoefficientIsSmall(
        const std::unordered_map<quake::GroundStation, double> &cumulative_keys) {
    for (const auto &record : cumulative_keys) {
        LOG_IF(WARNING, record.second < 0.01)
                        << "Distribution coefficient for " << record.first.name()
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

std::unordered_map<quake::GroundStation, double> NormalizeToWeight(const std::unordered_map<quake::GroundStation, double> &initial_weights,
                                                                   double normalized_weight) {
    double items_raw_weight = 0.0;
    for (const auto &entry : initial_weights) {
        CHECK_GE(entry.second, 0.0);
        items_raw_weight += entry.second;
    }

    std::unordered_map<quake::GroundStation, double> normalized_weights;
    for (const auto &entry : initial_weights) {
        normalized_weights.emplace(entry.first, entry.second / items_raw_weight * normalized_weight);
    }

    OutputWarningIfDistributionCoefficientIsSmall(normalized_weights);
    return normalized_weights;
}

std::unordered_map<quake::GroundStation, double> GetDistributionCoefficientsFromHouseholdsWithUltraFastBroadbandWithOverride(
        const std::vector<quake::GroundStation> &ground_stations) {
    static const std::unordered_map<quake::GroundStation, int64> PremisesWithUltraFastBroadbandAccess = {
            {quake::GroundStation::London,     5783941},
            {quake::GroundStation::Thurso,     385},
            {quake::GroundStation::Cambridge,  454174},
            {quake::GroundStation::Birmingham, 2512320},
            {quake::GroundStation::Glasgow,    1774439},
            {quake::GroundStation::Manchester, 1775340},
            {quake::GroundStation::Bristol,    1528836},
            {quake::GroundStation::Ipswich,    316515},
            {quake::GroundStation::York,       371431},
            {quake::GroundStation::Belfast,    1265413}
    };

    static const std::unordered_map<quake::GroundStation, double> PremisesWithWeightOverride{
            {quake::GroundStation::Thurso, 0.01}
    };

    double total_overwrite_weight = 0.0;
    std::unordered_map<quake::GroundStation, double> ground_station_raw_weights;
    for (const auto &ground_station : ground_stations) {
        const auto find_it = PremisesWithWeightOverride.find(ground_station);
        if (find_it != std::cend(PremisesWithWeightOverride)) {
            total_overwrite_weight += find_it->second;
        } else {
            ground_station_raw_weights.emplace(ground_station, PremisesWithUltraFastBroadbandAccess.at(ground_station));
        }
    }

    CHECK_LE(total_overwrite_weight, 1.0);

    // obtain normalized weights for the weights of the ground stations inferred from data
    auto normalized_weights = NormalizeToWeight(ground_station_raw_weights, 1.0 - total_overwrite_weight);

    // insert weights of ground stations with overwrite
    for (const auto &station: ground_stations) {
        if (normalized_weights.find(station) == std::cend(normalized_weights)) {
            normalized_weights.emplace(station, PremisesWithWeightOverride.at(station));
        }
    }

    return normalized_weights;
}

std::unordered_map<quake::GroundStation, double> GetDistributionCoefficientsFromPopulation(const std::vector<quake::GroundStation> &ground_stations) {
    static const std::unordered_map<quake::GroundStation, int64> Population = {
            {quake::GroundStation::London,     8825000},
            {quake::GroundStation::Thurso,     7933},
            {quake::GroundStation::Cambridge,  124900},
            {quake::GroundStation::Birmingham, 1137100},
            {quake::GroundStation::Glasgow,    621020},
            {quake::GroundStation::Manchester, 545500},
            {quake::GroundStation::Bristol,    459300},
            {quake::GroundStation::Ipswich,    133384},
            {quake::GroundStation::York,       208200},
            {quake::GroundStation::Belfast,    280211}
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
    std::unordered_map<quake::GroundStation, double> station_flow;
    for (auto station_index = 0; station_index < station_size; ++station_index) {
        station_flow.emplace(station.at(station_index),
                             std::accumulate(std::cbegin(flow_matrix[station_index]), std::cend(flow_matrix[station_index]), static_cast<int64>(0)));
    }

    return NormalizeToWeight(station_flow, 1.0);
}

std::unordered_map<quake::GroundStation, int> GetInitialBuffers(
        const std::unordered_map<quake::GroundStation, double> &distribution_coefficients,
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

std::unordered_map<quake::GroundStation, int> GetFixedInitialBuffers(
        const std::unordered_map<quake::GroundStation, double> &distribution_coefficients, int fixed_size) {
    CHECK_GT(fixed_size, 0);

    std::unordered_map<quake::GroundStation, int> initial_buffers;
    for (const auto &distribution_coefficient : distribution_coefficients) {
        initial_buffers.emplace(distribution_coefficient.first, fixed_size);
    }
    return initial_buffers;
}

std::unordered_map<quake::GroundStation, int> GetDailyKeyConsumption(
        const std::unordered_map<quake::GroundStation, int> &initial_buffers,
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

std::unordered_map<quake::GroundStation, int> GetNoKeyConsumption(
        const std::unordered_map<quake::GroundStation, int> &initial_buffers,
        int days) {
    CHECK_GT(days, 0);

    std::unordered_map<quake::GroundStation, int> key_consumption;
    for (const auto &initial_buffer : initial_buffers) {
        key_consumption.emplace(initial_buffer.first, 0);
    }
    return key_consumption;
}

struct ElevationToTransferIndex {
public:
    static const auto ANGLE_REF = 0;
    static const auto KEY_RATE_REF = 1;
    static const auto BIT_RATE_REF = 2;

    explicit ElevationToTransferIndex(std::vector<std::tuple<double, double, double> > elevation_to_transfer_index)
            : elevation_to_transfer_index_{std::move(elevation_to_transfer_index)} {}

    double KeyRate(double elevation) const { return Extract<KEY_RATE_REF>(elevation); }

    double BitRate(double elevation) const { return Extract<BIT_RATE_REF>(elevation); }

private:
    inline static bool ReferenceComparator(const std::tuple<double, double, double> &row, double angle) {
        return std::get<ANGLE_REF>(row) < angle;
    }

    template<unsigned int field>
    double Extract(double angle) const {
        if (angle < quake::util::MIN_ELEVATION) {
            return 0.0;
        }

        double transfer_rate = 0.0;
        auto lower_bound_it = std::lower_bound(std::cbegin(elevation_to_transfer_index_),
                                               std::cend(elevation_to_transfer_index_),
                                               angle,
                                               ReferenceComparator);
        if (lower_bound_it == std::cend(elevation_to_transfer_index_)) {
            // all elements are smaller than the reference
            transfer_rate
                    = std::get<field>(elevation_to_transfer_index_[elevation_to_transfer_index_.size() - 1]);
        } else if (lower_bound_it == std::cbegin(elevation_to_transfer_index_)) {
            // all elements are larger than the reference
            transfer_rate = std::get<field>(*lower_bound_it);
        } else {
            auto right_it = lower_bound_it;
            auto left_it = lower_bound_it - 1;
            const auto lower_bound_angle = std::get<ANGLE_REF>(*left_it);
            const auto upper_bound_angle = std::get<ANGLE_REF>(*right_it);
            if (angle - lower_bound_angle > upper_bound_angle - angle) {
                transfer_rate = std::get<field>(*right_it);
            } else {
                transfer_rate = std::get<field>(*left_it);
            }
        }
        return transfer_rate;
    }

    std::vector<std::tuple<double, double, double> > elevation_to_transfer_index_;
};

void GetElevationData(const std::vector<quake::GroundStation> &ground_stations,
                      double altitude,
                      double inclination,
                      double raan,
                      boost::posix_time::ptime initial_epoch,
                      boost::posix_time::time_period observation_period,
                      std::unordered_map<quake::GroundStation, std::vector<double> > &output_elevation_data,
                      std::vector<boost::posix_time::time_period> &output_umbra) {
    static const auto TIME_STEP = boost::posix_time::seconds(1);

    quake::KeplerElements initial_satellite_position{
            quake::util::EARTH_EQUATORIAL_RADIUS_KM + altitude,
            0.0,
            Util::DegreesToRadians(inclination),
            Util::DegreesToRadians(raan),
            0.0,
            Util::DegreesToRadians(46.0)
    };
    quake::SatelliteTracker tracker{quake::KeplerElements::DEFAULT, initial_epoch, observation_period, TIME_STEP};

    std::vector<Eci> satellite_positions;
    std::vector<boost::posix_time::time_period> umbras;
    std::vector<boost::posix_time::time_period> penumbras;
    tracker.CalculatePositions(satellite_positions, umbras, penumbras);

    std::unordered_map<quake::GroundStation, std::vector<double>> elevation_data;
    for (const auto &ground_station : ground_stations) {
        Observer ground_station_observer{ground_station.coordinates()};

        std::vector<double> elevation;
        elevation.reserve(satellite_positions.size());

        for (const auto &satellite_position : satellite_positions) {
            const auto elevation_radians = ground_station_observer.GetLookAngle(satellite_position).elevation;
            elevation.emplace_back(Util::RadiansToDegrees(elevation_radians));
        }

        elevation_data.emplace(ground_station, std::move(elevation));
    }

    output_elevation_data = elevation_data;
    output_umbra = umbras;
}

std::unordered_map<quake::GroundStation, std::unordered_map<boost::gregorian::date, std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > >
GetSunsetSunriseData(const quake::util::Resources &resources, const std::vector<quake::GroundStation> &ground_stations) {

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

    return sunset_sunrise_data;
}

std::vector<boost::posix_time::time_period> Intersect(const std::vector<boost::posix_time::time_period> &left,
                                                      const std::vector<boost::posix_time::time_period> &right) {
    std::vector<boost::posix_time::time_period> intersection_windows;

    auto left_window_it = std::cbegin(left);
    const auto left_window_end_it = std::cend(left);

    auto right_window_it = std::cbegin(right);
    const auto right_window_end_it = std::cend(right);
    for (; left_window_it != left_window_end_it && right_window_it != right_window_end_it;) {
        const boost::posix_time::time_period left_window = *left_window_it;
        const boost::posix_time::time_period right_window = *right_window_it;

        if (left_window.is_after(right_window.end())
            || (left_window.is_adjacent(right_window) && left_window.begin() == right_window.end())) {
            ++right_window_it;
            continue;
        } else if (left_window.is_before(right_window.begin())
                   || (left_window.is_adjacent(right_window) && left_window.end() == right_window.begin())) {
            ++left_window_it;
            continue;
        } else {
            CHECK(left_window.intersects(right_window) || right_window.contains(left_window));

            if (right_window.contains(left_window)) {
                intersection_windows.push_back(left_window);
            } else {
                auto intersection_window = left_window.intersection(right_window);
                if (intersection_window.length() > boost::posix_time::seconds(0)) {
                    intersection_windows.emplace_back(intersection_window);
                }
            }

            ++left_window_it;
        }
    }

    return intersection_windows;
}

std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period>> GetCommunicationWindowData(
        const std::vector<quake::GroundStation> &ground_stations,
        const quake::util::Resources &resources,
        const boost::posix_time::time_period &time_period,
        const std::unordered_map<quake::GroundStation, std::vector<double> > &elevation_data,
        const std::vector<boost::posix_time::time_period> &umbra_windows) {
    const auto sunset_sunrise_data = GetSunsetSunriseData(resources, ground_stations);

    const auto start_time = time_period.begin();
    std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period>> transfer_window_data;
    for (const auto &ground_station : ground_stations) {
        const auto &ground_station_elevation = elevation_data.at(ground_station);

        std::vector<boost::posix_time::time_period> elevation_windows;
        const auto elevation_size = ground_station_elevation.size();
        for (auto begin_transfer_delta = 0; begin_transfer_delta < elevation_size; ++begin_transfer_delta) {
            if (can_transfer(ground_station_elevation[begin_transfer_delta])) {
                auto end_transfer_delta = begin_transfer_delta + 1;
                for (; end_transfer_delta < elevation_size
                       && can_transfer(ground_station_elevation[end_transfer_delta]);
                       ++end_transfer_delta);

                boost::posix_time::time_period elevation_window{start_time + boost::posix_time::seconds(begin_transfer_delta),
                                                                boost::posix_time::seconds(end_transfer_delta - begin_transfer_delta)};
                elevation_windows.emplace_back(elevation_window);
                begin_transfer_delta = end_transfer_delta;
            }
        }

        std::vector<boost::posix_time::time_period> night_windows;
        for (auto current_day = start_time.date(); current_day < time_period.end().date();) {
            const auto next_day = current_day + boost::gregorian::days(1);
            const auto sunset_it = sunset_sunrise_data.at(ground_station).find(current_day);
            if (sunset_it == std::end(sunset_sunrise_data.at(ground_station))) {
                std::stringstream msg;
                msg << "Sunset not known for " << ground_station << " for day " << start_time.date();
                throw std::runtime_error(msg.str());
            }

            const auto sunrise_it = sunset_sunrise_data.at(ground_station).find(next_day);
            if (sunset_it == std::end(sunset_sunrise_data.at(ground_station))) {
                std::stringstream msg;
                msg << "Sunset not known for " << ground_station << " for day " << start_time.date();
                throw std::runtime_error(msg.str());
            }

            const auto sunset = sunset_it->second.second;
            const auto sunrise = sunrise_it->second.first;
            night_windows.emplace_back(sunset, sunrise);

            current_day = next_day;
        }

        std::vector<boost::posix_time::time_period> initial_transfer_windows = Intersect(elevation_windows, night_windows);
        std::vector<boost::posix_time::time_period> final_transfer_windows = Intersect(initial_transfer_windows, umbra_windows);
        transfer_window_data.emplace(ground_station, std::move(final_transfer_windows));
    }

    return transfer_window_data;
}

quake::Problem quake::ProblemGenerator::Create(std::vector<quake::GroundStation> ground_stations,
                                               boost::posix_time::ptime initial_epoch,
                                               boost::posix_time::time_period time_period) const {
    quake::util::Resources resources{"~/dev/quake/data"};

    std::unordered_map<GroundStation, std::vector<double> > elevation_data;
    std::vector<boost::posix_time::time_period> umbra_windows;
    GetElevationData(ground_stations,
                     KeplerElements::DEFAULT_ALTITUDE,
                     KeplerElements::DEFAULT_INCLINATION,
                     KeplerElements::DEFAULT_RAAN,
                     initial_epoch,
                     time_period,
                     elevation_data,
                     umbra_windows);
    auto communication_window_data = GetCommunicationWindowData(ground_stations, resources, time_period, elevation_data, umbra_windows);

    const auto length_days = time_period.length().hours() / 24;
    auto distribution_coefficients = GetLinearDistributionCoefficients(ground_stations);
    auto initial_buffers = GetInitialBuffers(distribution_coefficients, length_days);
    auto key_consumption = GetDailyKeyConsumption(initial_buffers, length_days);

    if (VLOG_IS_ON(1)) {
        std::ostringstream output;

        output << "Generated problem: " << std::endl;
        for (const auto &station_data : communication_window_data) {
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
            std::move(communication_window_data),
            std::move(elevation_data),
            std::move(distribution_coefficients),
            std::move(initial_buffers),
            std::move(key_consumption),
            std::move(transfer_rate_data),
            DEFAULT_SWITCH_DURATION};
}

quake::ExtendedProblem quake::ProblemGenerator::CreateExtendedProblem(const std::vector<quake::GroundStation> &ground_stations,
                                                                      double altitude,
                                                                      double inclination,
                                                                      double raan,
                                                                      boost::posix_time::ptime initial_epoch,
                                                                      boost::posix_time::time_period time_period) const {
    quake::util::Resources resources{"~/dev/quake/data"};

    std::unordered_map<GroundStation, std::vector<double> > elevation_data;
    std::vector<boost::posix_time::time_period> umbra_windows;
    GetElevationData(ground_stations, altitude, inclination, raan, initial_epoch, time_period, elevation_data, umbra_windows);
    const auto communication_window_data = GetCommunicationWindowData(ground_stations, resources, time_period, elevation_data, umbra_windows);

    quake::TransferRateReader transfer_rate_reader;
    auto transfer_rate_data = transfer_rate_reader.Read(resources.TransferRate(DEFAULT_TRANSFER_RATE));
    ElevationToTransferIndex elevation_transfer_index(std::move(transfer_rate_data));

    const auto length_days = time_period.length().hours() / 24;

    const auto distribution_coefficients = GetDistributionCoefficientsFromHouseholdsWithUltraFastBroadbandWithOverride(ground_stations);
    const auto initial_buffers = GetFixedInitialBuffers(distribution_coefficients, 64);
    const auto key_consumption = GetNoKeyConsumption(initial_buffers, length_days);

    const auto start_time = time_period.begin();
    std::vector<ExtendedProblem::StationData> station_data;
    station_data.reserve(ground_stations.size());
    for (const auto &ground_station: ground_stations) {
        std::vector<ExtendedProblem::CommunicationWindowData> communication_windows;
        const auto &station_elevation_data = elevation_data.at(ground_station);
        for (const auto &transfer_window : communication_window_data.at(ground_station)) {
            const auto begin_elevation_index = (transfer_window.begin() - start_time).total_seconds();
            const auto end_elevation_index = (transfer_window.end() - start_time).total_seconds();

            const auto window_duration = transfer_window.length().total_seconds();
            std::vector<double> elevation_angles;
            elevation_angles.reserve(window_duration);
            std::copy(std::cbegin(station_elevation_data) + begin_elevation_index,
                      std::cbegin(station_elevation_data) + end_elevation_index,
                      std::back_inserter(elevation_angles));

            std::vector<double> transfer_rates;
            transfer_rates.reserve(window_duration);
            for (const auto angle : elevation_angles) {
                CHECK_GE(angle, util::MIN_ELEVATION);
                transfer_rates.emplace_back(elevation_transfer_index.KeyRate(angle));
            }

            communication_windows.emplace_back(transfer_window, std::move(elevation_angles), std::move(transfer_rates));
        }

        station_data.emplace_back(ground_station,
                                  distribution_coefficients.at(ground_station),
                                  initial_buffers.at(ground_station),
                                  key_consumption.at(ground_station),
                                  std::move(communication_windows));
    }

    if (VLOG_IS_ON(1)) {
        std::ostringstream output;

        output << "Generated problem: " << std::endl;
        for (const auto &local_data : station_data) {
            output << "Station: " << local_data.Station << std::endl;
            output << " - Transfer windows: ";

            auto window_it = std::cbegin(local_data.CommunicationWindows);
            const auto window_end_it = std::cend(local_data.CommunicationWindows);
            if (window_it != window_end_it) {
                output << window_it->Period;
                ++window_it;
            }

            for (; window_it != window_end_it; ++window_it) {
                output << " ";
                output << window_it->Period;
            }
            output << std::endl;
            output << " - Distribution coefficient: " << local_data.TransferShare << std::endl;
        }

        VLOG(1) << output.str();
    }

    return {time_period, DEFAULT_SWITCH_DURATION, std::move(station_data)};
}
