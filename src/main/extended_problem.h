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
                                    std::vector<double> elevation_angles,
                                    std::vector<double>
                                    transfer_rates)
                    : Period{period},
                      ElevationAngles{std::move(elevation_angles)},
                      TransferRates{std::move(transfer_rates)} {}

            boost::posix_time::time_period Period;
            std::vector<double> ElevationAngles;
            std::vector<double> TransferRates;
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
    };
}


#endif //QUAKE_EXTENDED_PROBLEM_H
