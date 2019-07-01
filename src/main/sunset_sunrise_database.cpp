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

#include <glog/logging.h>

#include <boost/date_time/local_time/local_time.hpp>

#include <date/tz.h>

#include "sunset_sunrise_database.h"

#include "util/resources.h"
#include "sunset_sunrise_reader.h"

void quake::SunsetSunriseDatabase::Load(const std::vector<quake::GroundStation> &stations) {
    database_.clear();

    quake::util::Resources resources{"~/dev/quake/data"};

    quake::SunsetSunriseReader reader{quake::util::Resources::DEFAULT_LOCAL_TIME_ZONE};
    for (const auto &station: stations) {
        const auto records_pair = reader.Read(resources.SunsetSunriseData(station));

        for (const auto &row : records_pair.second) {
            CHECK_EQ(row.first.date(), row.second.date());

            const auto find_it = database_.find(row.first.date());
            if (find_it == std::cend(database_)) {
                database_.emplace(
                        row.first.date(),
                        std::unordered_map<GroundStation,
                                std::pair<boost::posix_time::ptime, boost::posix_time::ptime> >());
            }

            database_[row.first.date()].emplace(records_pair.first, row);
        }
    }
}

boost::posix_time::ptime quake::SunsetSunriseDatabase::MinSunset(const boost::gregorian::date &date) const {
    static const auto END_DURATION = boost::posix_time::time_duration(23, 59, 59);

    const auto find_it = database_.find(date);
    CHECK(find_it != std::cend(database_));
    const auto &stations_map = find_it->second;

    boost::posix_time::ptime min_sunset(date, END_DURATION);
    for (const auto &row : stations_map) {
        const auto &local_sunset = row.second.second;
        if (local_sunset < min_sunset) {
            min_sunset = local_sunset;
        }
    }
    CHECK_NE(min_sunset.time_of_day(), END_DURATION);
    return min_sunset;
}
