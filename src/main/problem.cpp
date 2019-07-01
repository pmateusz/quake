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

#include <algorithm>

#include <glog/logging.h>

#include "util/constants.h"
#include "util/json.h"

#include "problem.h"

const int quake::Problem::ANGLE_REF = 0;
const int quake::Problem::KEY_RATE_REF = 1;
const int quake::Problem::BIT_RATE_REF = 2;

inline static bool ReferenceComparator(const std::tuple<double, double, double> &row, double reference) {
    return std::get<quake::Problem::ANGLE_REF>(row) < reference;
}

quake::Problem::Problem(boost::posix_time::time_period observation_period,
                        std::vector<quake::GroundStation> ground_stations,
                        std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > transfer_windows,
                        std::unordered_map<quake::GroundStation, std::vector<double> > elevation_angles,
                        std::unordered_map<quake::GroundStation, double> transfer_share,
                        std::unordered_map<quake::GroundStation, int> initial_buffers,
                        std::unordered_map<quake::GroundStation, int> key_consumption,
                        std::vector<std::tuple<double, double, double> > reference_transfer_rates,
                        boost::posix_time::time_duration switch_duration)
        : observation_period_{observation_period},
          switch_duration_{std::move(switch_duration)},
          ground_stations_{std::move(ground_stations)},
          transfer_windows_{std::move(transfer_windows)},
          elevation_angles_{std::move(elevation_angles)},
          transfer_share_{std::move(transfer_share)},
          initial_buffers_{std::move(initial_buffers)},
          key_consumption_{std::move(key_consumption)},
          reference_transfer_rates_{std::move(reference_transfer_rates)},
          transfer_rates_{} {

    for (const auto &ground_station : ground_stations_) {
        const auto &ground_station_elevation = elevation_angles_.at(ground_station);
        const auto max_time_dim = ground_station_elevation.size();
        std::vector<double> transfer_rates(max_time_dim);

        for (auto time_index = 0; time_index < max_time_dim; ++time_index) {
            const auto current_elevation_angle = ground_station_elevation[time_index];

            double transfer_rate = 0.0;
            if (current_elevation_angle >= util::MIN_ELEVATION) {
                auto lower_bound_it = std::lower_bound(std::cbegin(reference_transfer_rates_),
                                                       std::cend(reference_transfer_rates_),
                                                       current_elevation_angle,
                                                       ReferenceComparator);
                if (lower_bound_it == std::cend(reference_transfer_rates_)) {
                    // all elements are smaller than the reference
                    transfer_rate
                            = std::get<KEY_RATE_REF>(reference_transfer_rates_[reference_transfer_rates_.size() - 1]);
                } else if (lower_bound_it == std::cbegin(reference_transfer_rates_)) {
                    // all elements are larger than the reference
                    transfer_rate = std::get<KEY_RATE_REF>(*lower_bound_it);
                } else {
                    auto right_it = lower_bound_it;
                    auto left_it = lower_bound_it - 1;
                    const auto lower_bound_angle = std::get<ANGLE_REF>(*left_it);
                    const auto upper_bound_angle = std::get<ANGLE_REF>(*right_it);
                    if (current_elevation_angle - lower_bound_angle >
                        upper_bound_angle - current_elevation_angle) {
                        transfer_rate = std::get<KEY_RATE_REF>(*right_it);
                    } else {
                        transfer_rate = std::get<KEY_RATE_REF>(*left_it);
                    }
                }
            }

            transfer_rates[time_index] = transfer_rate;
        }

        transfer_rates_.emplace(ground_station, std::move(transfer_rates));
    }
}

double quake::Problem::KeyRate(const quake::GroundStation &station,
                               const boost::posix_time::ptime &datetime) const {
    const auto offset = (datetime - this->observation_period_.begin()).total_seconds();
    return this->transfer_rates_.find(station)->second.at(offset);
}

double
quake::Problem::ElevationAngle(const quake::GroundStation &station,
                               const boost::posix_time::ptime &datetime) const {
    const auto offset = (datetime - this->observation_period_.begin()).total_seconds();
    return this->elevation_angles_.find(station)->second.at(offset);
}

double quake::Problem::TransferKeyRate(const quake::GroundStation &station,
                                       const boost::posix_time::ptime &datetime) const {
    const auto transfer_windows_it = transfer_windows_.find(station);
    if (transfer_windows_it == std::cend(transfer_windows_)) {
        return 0;
    }

    for (const auto &transfer_window : transfer_windows_it->second) {
        if (transfer_window.contains(datetime) || transfer_window.end() == datetime) {
            return KeyRate(station, datetime);
        }

        if (datetime < transfer_window.begin()) {
            break;
        }
    }

    return 0.0;
}

void quake::to_json(nlohmann::json &json, const quake::Problem &problem) {
    nlohmann::json object;

    object["observation_period"] = problem.observation_period_;
    object["switch_duration"] = problem.switch_duration_;
    object["ground_stations"] = problem.ground_stations_;
    object["transfer_windows"] = problem.transfer_windows_;
    object["elevation_angles"] = problem.elevation_angles_;
    object["transfer_rates"] = problem.transfer_rates_;
    object["transfer_share"] = problem.transfer_share_;
    object["initial_buffers"] = problem.initial_buffers_;
    object["key_consumption"] = problem.key_consumption_;
    object["reference_transfer_rates"] = problem.reference_transfer_rates_;

    json = object;
}


quake::Problem quake::Problem::Round(std::size_t precision) const {
    const auto &round_number = [&precision](double value) -> double {
        static const auto factor = pow(10.0, precision);
        return round(value * factor) / factor;
    };

    std::unordered_map<quake::GroundStation, std::vector<double> > elevation_angles_rounded;
    for (const auto &station_angles : elevation_angles_) {
        std::vector<double> rounded_vector;
        rounded_vector.reserve(station_angles.second.size());
        for (const auto number : station_angles.second) {
            rounded_vector.emplace_back(round_number(number));
        }

        elevation_angles_rounded.emplace(station_angles.first, std::move(rounded_vector));
    }

    std::unordered_map<quake::GroundStation, double> transfer_share_rounded;
    for (const auto &station_transfer : this->transfer_share_) {
        transfer_share_rounded.emplace(station_transfer.first, round_number(station_transfer.second));
    }

    std::vector<std::tuple<double, double, double> > reference_transfer_rates_rounded;
    reference_transfer_rates_rounded.reserve(this->reference_transfer_rates_.size());
    for (const auto &reference_transfer_rate : reference_transfer_rates_) {
        reference_transfer_rates_rounded.emplace_back(
                round_number(std::get<0>(reference_transfer_rate)),
                round_number(std::get<1>(reference_transfer_rate)),
                round_number(std::get<2>(reference_transfer_rate)));
    }

    return quake::Problem(observation_period_,
                          ground_stations_,
                          transfer_windows_,
                          std::move(elevation_angles_rounded),
                          std::move(transfer_share_rounded),
                          initial_buffers_,
                          key_consumption_,
                          std::move(reference_transfer_rates_rounded),
                          switch_duration_);
}
