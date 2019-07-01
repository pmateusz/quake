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

#ifndef QUAKE_PROBLEM_H
#define QUAKE_PROBLEM_H

#include <vector>
#include <utility>
#include <unordered_map>
#include <tuple>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include <nlohmann/json.hpp>

#include "ground_station.h"

namespace quake {

    class Problem {
    public:
        static const int ANGLE_REF;
        static const int KEY_RATE_REF;
        static const int BIT_RATE_REF;

        Problem(boost::posix_time::time_period observation_period,
                std::vector<GroundStation> ground_stations,
                std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > transfer_windows,
                std::unordered_map<quake::GroundStation, std::vector<double> > elevation_angles,
                std::unordered_map<quake::GroundStation, double> transfer_share,
                std::unordered_map<quake::GroundStation, int> initial_buffers,
                std::unordered_map<quake::GroundStation, int> key_consumption,
                std::vector<std::tuple<double, double, double> > reference_transfer_rates,
                boost::posix_time::time_duration switch_duration);

        Problem Round(std::size_t precision) const;

        inline const std::vector<GroundStation> &GroundStations() const {
            return ground_stations_;
        }

        inline const std::vector<boost::posix_time::time_period> &TransferWindows(
                const GroundStation &ground_station) const {
            return transfer_windows_.at(ground_station);
        }

        inline double TransferShare(const GroundStation &ground_station) const {
            return transfer_share_.at(ground_station);
        }

        inline int InitialBuffer(const GroundStation &ground_station) const {
            return initial_buffers_.at(ground_station);
        }

        inline int KeyConsumption(const GroundStation &ground_station) const {
            return key_consumption_.at(ground_station);
        }

        inline boost::posix_time::ptime StartTime() const {
            return observation_period_.begin();
        }

        inline boost::posix_time::time_duration SwitchDuration() const {
            return switch_duration_;
        }

        double TransferKeyRate(const GroundStation &station, const boost::posix_time::ptime &datetime) const;

        double KeyRate(const GroundStation &station, const boost::posix_time::ptime &datetime) const;

        double ElevationAngle(const GroundStation &station, const boost::posix_time::ptime &datetime) const;

    private:
        friend void to_json(nlohmann::json &json, const Problem &problem);

        boost::posix_time::time_period observation_period_;
        boost::posix_time::time_duration switch_duration_;

        std::vector<GroundStation> ground_stations_;
        std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > transfer_windows_;
        std::unordered_map<quake::GroundStation, std::vector<double> > elevation_angles_;
        std::unordered_map<quake::GroundStation, std::vector<double> > transfer_rates_;
        std::unordered_map<quake::GroundStation, double> transfer_share_;
        std::unordered_map<quake::GroundStation, int> initial_buffers_;
        std::unordered_map<quake::GroundStation, int> key_consumption_;
        std::vector<std::tuple<double, double, double> > reference_transfer_rates_;
    };

    void to_json(nlohmann::json &json, const Problem &problem);
}

#endif //QUAKE_PROBLEM_H
