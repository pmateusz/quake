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

#include <vector>
#include <unordered_map>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include "util/hash.h"
#include "ground_station.h"

#ifndef QUAKE_SUNSET_SUNRISE_DATABASE_H
#define QUAKE_SUNSET_SUNRISE_DATABASE_H

namespace quake {

    class SunsetSunriseDatabase {
    public:
        void Load(const std::vector<GroundStation> &stations);

        boost::posix_time::ptime MinSunset(const boost::gregorian::date &date) const;

    private:
        std::unordered_map<boost::gregorian::date,
                std::unordered_map<GroundStation,
                        std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > > database_;
    };
}


#endif //QUAKE_SUNSET_SUNRISE_DATABASE_H
