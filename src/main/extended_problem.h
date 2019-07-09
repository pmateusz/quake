#include <utility>

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

#ifndef QUAKE_EXTENDED_PROBLEM_H
#define QUAKE_EXTENDED_PROBLEM_H

#include <vector>
#include <unordered_map>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include <nlohmann/json.hpp>

#include "util/json.h"
#include "ground_station.h"

// TODO: serialize extended problem to json

namespace quake {

    class ExtendedProblem {
    public:
        struct MetaData {
            MetaData(boost::posix_time::time_period period, boost::posix_time::time_duration switch_duration)
                    : Period(period),
                      SwitchDuration(std::move(switch_duration)) {}

            boost::posix_time::time_period Period;
            boost::posix_time::time_duration SwitchDuration;
        };

        struct CommunicationWindowData {
            CommunicationWindowData()
                    : CommunicationWindowData(boost::posix_time::time_period(boost::posix_time::ptime(),
                                                                             boost::posix_time::seconds(0)),
                                              std::vector<double>{},
                                              std::vector<double>{}) {}

            CommunicationWindowData(boost::posix_time::time_period period,
                                    std::vector<double> elevation_angle,
                                    std::vector<double> key_rate)
                    : Period{period},
                      ElevationAngle{std::move(elevation_angle)},
                      KeyRate{std::move(key_rate)} {}

            boost::posix_time::time_period Period;
            std::vector<double> ElevationAngle;
            std::vector<double> KeyRate;
        };

        struct StationData {
            StationData()
                    : StationData(GroundStation::None, 0.0, 0, 0, std::vector<CommunicationWindowData>{}) {}

            StationData(GroundStation station,
                        double transfer_share,
                        int initial_buffer,
                        int key_consumption,
                        std::vector<CommunicationWindowData> communication_windows)
                    : Station{std::move(station)},
                      TransferShare{transfer_share},
                      InitialBuffer{initial_buffer},
                      KeyConsumption{key_consumption},
                      CommunicationWindows{std::move(communication_windows)} {}

            GroundStation Station;
            double TransferShare;
            int InitialBuffer;
            int KeyConsumption;
            std::vector<CommunicationWindowData> CommunicationWindows;
        };

        ExtendedProblem(MetaData metadata, std::vector<StationData> station_data);

        ExtendedProblem Round(unsigned int decimal_places) const;

        inline const std::vector<GroundStation> &GroundStations() const { return ground_stations_; }

        std::vector<boost::posix_time::time_period> TransferWindows(const GroundStation &ground_station) const;

        inline double TransferShare(const GroundStation &ground_station) { return GetStationData(ground_station).TransferShare; }

        inline int InitialBuffer(const GroundStation &ground_station) const { return GetStationData(ground_station).InitialBuffer; }

        inline int KeyConsumption(const GroundStation &ground_station) const { return GetStationData(ground_station).KeyConsumption; }

        inline boost::posix_time::ptime StartTime() const { return metadata_.Period.begin(); }

        inline boost::posix_time::time_duration SwitchDuration() const { return metadata_.SwitchDuration; }

        double KeyRate(const GroundStation &station, const boost::posix_time::ptime &datetime) const;

        double ElevationAngle(const GroundStation &station, const boost::posix_time::ptime &datetime) const;

    private:
        const StationData &GetStationData(const quake::GroundStation &station) const;

        friend void to_json(nlohmann::json &json, const ExtendedProblem &problem);

        MetaData metadata_;
        std::vector<StationData> station_data_;
        std::vector<GroundStation> ground_stations_;
    };

    void to_json(nlohmann::json &json, const ExtendedProblem &problem);

    void to_json(nlohmann::json &json, const ExtendedProblem::MetaData &metadata);

    void to_json(nlohmann::json &json, const ExtendedProblem::StationData &station_data);

    void to_json(nlohmann::json &json, const ExtendedProblem::CommunicationWindowData &communication_window_data);
}


#endif //QUAKE_EXTENDED_PROBLEM_H
