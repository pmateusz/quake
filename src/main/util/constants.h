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

#ifndef QUAKE_CONSTANTS_H
#define QUAKE_CONSTANTS_H

#include <cmath>

#include <pykep/third_party/libsgp4/Globals.h>

namespace quake {

    namespace util {
        static const auto ASTRONOMIC_UNIT_KM = 149597870.700;
        static const auto EARTH_EQUATORIAL_RADIUS_KM = 6378.1363;
        static const auto EARTH_POLAR_RADIUS_KM = 6356.751;
        static const auto SOLAR_RADIUS_KM = 696000;
        static const auto NODAL_PRECESSION_EARTH = 1.0826E-3;
        static const auto EARTH_STANDARD_GRAVITATIONAL_PARAM = kMU;
        static const auto SECONDS_PER_DAY = 24 * 60 * 60;
        static const auto EARTH_ECCENTRICITY
                = sqrt(1.0 - pow(EARTH_POLAR_RADIUS_KM / EARTH_EQUATORIAL_RADIUS_KM, 2.0));
        static const auto ECCENTRICITY_TOLERANCE = 1.0e-10;
        static const auto MIN_ELEVATION = 15.0;
    }
}
#endif //QUAKE_CONSTANTS_H
