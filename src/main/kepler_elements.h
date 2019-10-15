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

#ifndef QUAKE_KEPLER_ELEMENTS_H
#define QUAKE_KEPLER_ELEMENTS_H

#include <cmath>

#include "util/constants.h"

namespace quake {

    class KeplerElements {
    public:
        static constexpr double DEFAULT_INCLINATION = 97.658190099944605;
        static constexpr double DEFAULT_ALTITUDE = 566.897046176199410;
        static constexpr double DEFAULT_RAAN = 110.5;

        static const KeplerElements DEFAULT;

        static double GetSunSynchronousInclination(double semimajor_axis, double eccentricity);

        KeplerElements(double semimajor_axis,
                       double eccentricity,
                       double inclination,
                       double ascending_node_longitude,
                       double periapsis_argument,
                       double true_anomaly);

        inline double SemimajorAxis() const noexcept {
            return semimajor_axis_;
        }

        inline double Eccentricity() const noexcept {
            return eccentricity_;
        }

        inline double Inclination() const noexcept {
            return inclination_;
        }

        inline double AscendingNodeLongitude() const noexcept {
            return ascending_node_longitude_;
        }

        inline double PeriapsisArgument() const noexcept {
            return periapsis_argument_;
        }

        inline double TrueAnomaly() const noexcept {
            return true_anomaly_;
        }

        inline void SetTrueAnomaly(double value) noexcept {
            true_anomaly_ = value;
        }

        inline double MeanMotion() const noexcept {
            return mean_motion_;
        }

        inline double OrbitalPeriod() const noexcept {
            return orbital_period_;
        }

        inline double CircularOrbitVelocity() const noexcept {
            return circular_orbit_velocity_;
        }

        inline double SemilatusRectum() const noexcept {
            return semilatus_rectum_;
        }

        inline double AscendingNodeLongitudeVariation() const noexcept {
            return ascending_node_longitude_variation_;
        }

    private:
        double semimajor_axis_;
        double eccentricity_;
        double inclination_;
        double ascending_node_longitude_;
        double periapsis_argument_;
        double true_anomaly_;

        double mean_motion_;
        double orbital_period_;
        double circular_orbit_velocity_;
        double semilatus_rectum_;
        double ascending_node_longitude_variation_;
    };
}


#endif //QUAKE_KEPLER_ELEMENTS_H
