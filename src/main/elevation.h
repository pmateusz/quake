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

#ifndef QUAKE_ELEVATION_H
#define QUAKE_ELEVATION_H

#include <vector>

#include <boost/date_time.hpp>

namespace quake {

    class GroundStation;

    class KeplerElements;

    /**
    * Compute elevation angle using Pykep
    */
    std::vector<double> GetElevation(const GroundStation &ground_station,
                                     const KeplerElements &initial_satellite_position,
                                     const boost::posix_time::ptime &initial_epoch,
                                     const boost::posix_time::time_period &observation_period,
                                     const boost::posix_time::time_duration &time_step);

    /**
    * Compute elevation angle using a the algorithm of Marilena Di Carlo
    */
    std::vector<double> GetMatlabElevation(const GroundStation &ground_station,
                                           const KeplerElements &initial_satellite_position,
                                           const boost::posix_time::ptime &start_time,
                                           const boost::posix_time::time_duration &observation_time,
                                           const boost::posix_time::time_duration &time_step);
}


#endif //QUAKE_ELEVATION_H
