//
// Copyright 2018 Mateusz Polnik, Annalisa Riccardi
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

#include <cmath>
#include <glog/logging.h>

#include "util/constants.h"

#include "kepler_elements.h"

quake::KeplerElements::KeplerElements(double semi_major_axis,
                                      double eccentricity,
                                      double inclination,
                                      double right_ascension_ascending_node,
                                      double argument_of_perigee,
                                      double true_anomaly)
        : semimajor_axis_{semi_major_axis},
          eccentricity_{eccentricity},
          inclination_{inclination},
          ascending_node_longitude_{right_ascension_ascending_node},
          periapsis_argument_{argument_of_perigee},
          true_anomaly_{true_anomaly},
          mean_motion_{sqrt(util::EARTH_STANDARD_GRAVITATIONAL_PARAM / pow(semimajor_axis_, 3.0))},
          orbital_period_{2 * M_PI / mean_motion_},
          circular_orbit_velocity_{2 * M_PI / orbital_period_},
          semilatus_rectum_{semimajor_axis_ * (1.0 - pow(eccentricity_, 2.0))},
          ascending_node_longitude_variation_{-1.5
                                              * mean_motion_
                                              * util::J2
                                              * pow(util::EARTH_EQUATORIAL_RADIUS_KM / semilatus_rectum_, 2.0)
                                              * cos(inclination_)} {
    CHECK_NEAR(ascending_node_longitude_variation_,
               -3.0 / 2.0
               * quake::util::J2
               * pow(quake::util::EARTH_EQUATORIAL_RADIUS_KM / (semimajor_axis_ * (1 - pow(eccentricity_, 2.0))), 2.0)
               * sqrt(quake::util::EARTH_STANDARD_GRAVITATIONAL_PARAM / pow(semimajor_axis_, 3.0))
               * cos(inclination_), 0.00001); // compare RAAN drift
}

double quake::KeplerElements::GetSunSynchronousInclination(double semimajor_axis, double eccentricity) {
    return acos(-2.0 / 3.0
                * quake::util::RAAN_SSO_DRIFT_PER_YEAR
                * (1.0 / quake::util::J2)
                * pow((semimajor_axis * (1.0 - pow(eccentricity, 2.0)) / quake::util::EARTH_EQUATORIAL_RADIUS_KM), 2.0)
                * sqrt(pow(semimajor_axis, 3.0) / quake::util::EARTH_STANDARD_GRAVITATIONAL_PARAM));
}

// most_effective_configurations:
// - sma: 566.896708692896, raan 118.0 ta 280 - cross over England
// - sma: 566.896708692896, raan 107.0 ta 280 - two bands symmetric around midnight one cross
// - sma: 566.896708692896, raan 107.0 ta  42 - two bands one centered around midnight
// - sma: 566.896708692896, raan 110.5 ta  50 - two bands, zenith over London at midnight
// - sma: 500.000000000000, raan 110.5 ta  46

// parameters of the Sun Synchronous Orbit
const quake::KeplerElements quake::KeplerElements::DEFAULT{
        util::EARTH_EQUATORIAL_RADIUS_KM + DEFAULT_ALTITUDE,
        0,
        DEFAULT_INCLINATION * M_PI / 180.0,
        DEFAULT_RAAN * M_PI / 180.0,
        0.0,
        46.0 * M_PI / 180.0};
