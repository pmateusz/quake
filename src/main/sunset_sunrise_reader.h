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

#ifndef QUAKE_SUNSET_SUNRISE_LOADER_H
#define QUAKE_SUNSET_SUNRISE_LOADER_H

#include <fstream>
#include <string>
#include <regex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

#include <date/tz.h>

#include "ground_station.h"

namespace quake {

    class SunsetSunriseReader {
    public:
        explicit SunsetSunriseReader(std::string local_time_zone_name);

        std::pair<GroundStation, std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > > Read(
                const boost::filesystem::path &file_path) const;

    private:
        static const std::regex GROUND_STATION_PATTERN;
        static const std::regex FIRST_6_HEADER_MONTHS_PATTERN;
        static const std::regex EMPTY_LINE_PATTERN;
        static const std::regex SECOND_6_MONTHS_HEADER_PATTERN;
        static const std::regex HOUR_MINUTE_HEADER_PATTERN;
        static const std::regex RISE_SET_HEADER_PATTERN;
        static const std::regex TIMES_FORMAT_PATTERN;

        static const std::unordered_map<std::string, GroundStation> GROUND_STATIONS_BY_NAME;

        std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> >
        ReadSunsetSunrise(std::ifstream &stream, int year) const;

        std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> >
        ReadSunsetSunrise(std::ifstream &stream, int first_month, int last_month, int year) const;

        std::pair<boost::posix_time::ptime, boost::posix_time::ptime> ReadDay(int day,
                                                                              int month,
                                                                              int year,
                                                                              std::istringstream &stream) const;

        std::string local_time_zone_name_;
        date::time_zone const *local_time_zone_ptr_;
        date::time_zone const *utc_time_zone_ptr_;
    };
}


#endif //QUAKE_SUNSET_SUNRISE_LOADER_H
