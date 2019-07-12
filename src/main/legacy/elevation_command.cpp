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
#include <utility>

#include <glog/logging.h>

#include "elevation_command.h"

#include "util/math.h"
#include "ground_station.h"
#include "kepler_elements.h"
#include "elevation.h"

quake::ElevationCommand::ElevationCommand(boost::posix_time::time_period time_period,
                                          boost::filesystem::path output_file_path)
        : time_period_(time_period),
          output_file_path_(std::move(output_file_path)) {}

void quake::ElevationCommand::Run() {
    static const auto TIME_STEP = boost::posix_time::seconds(1);

    std::vector<GroundStation> ground_stations{
            GroundStation::London,
            GroundStation::Birmingham,
            GroundStation::Bristol,
            GroundStation::Cambridge,
            GroundStation::Glasgow,
            GroundStation::Ipswich,
            GroundStation::Manchester,
            GroundStation::Thurso,
            GroundStation::York
    };

    const boost::posix_time::ptime initial_epoch(boost::gregorian::date(time_period_.begin().date().year(), 1, 1),
                                                 boost::posix_time::seconds(0));

    std::vector<std::vector<double>> elevation_angles;
    for (const auto &ground_station : ground_stations) {
        elevation_angles.push_back(
                GetElevation(
                        ground_station,
                        KeplerElements::DEFAULT,
                        initial_epoch,
                        time_period_,
                        TIME_STEP));
    }

    std::ofstream output_file;
    output_file.open(output_file_path_.string(), std::ofstream::trunc);
    if (!output_file.is_open()) {
        std::stringstream error_msg;
        error_msg << "Filed to open the output file: " << output_file_path_ << ".";
        throw std::runtime_error(error_msg.str());
    }

    output_file << "# Time";
    for (const auto &ground_station : ground_stations) {
        output_file << ", ";
        output_file << ground_station.name();
    }
    output_file << std::endl;

    auto elevation_angle_size = elevation_angles.size();
    for (auto station_index = 0; station_index < elevation_angle_size; ++station_index) {
        const auto max_element = std::max_element(std::cbegin(elevation_angles[station_index]),
                                                  std::cend(elevation_angles[station_index]));
        const auto max_value = util::degrees(*max_element);
        LOG_IF(WARNING, max_value <= 10.0)
                        << "The largest elevation angle " << max_value
                        << " is below 10.0 for station " << ground_stations[station_index].name();
    }

    for (auto current_time = time_period_.begin(); current_time < time_period_.end(); current_time += TIME_STEP) {
        const auto elevation_index = (current_time - time_period_.begin()).total_seconds();
        output_file << current_time;
        for (const auto &elevation_vector : elevation_angles) {
            output_file << ", " << elevation_vector.at(elevation_index);
        }
        output_file << std::endl;
    }

    output_file.close();
}
