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

#ifndef QUAKE_MATH_H
#define QUAKE_MATH_H

#include <boost/config.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace quake {

    namespace util {

        const auto BINARY_PRECISION = 0.0001;

        boost::numeric::ublas::vector<double> mean(const boost::numeric::ublas::matrix<double> &matrix);

        boost::numeric::ublas::matrix<double> covariance(const boost::numeric::ublas::matrix<double> &matrix);

        inline double degrees(double radians) {
            return radians * boost::math::constants::radian<double>();
        }

        inline bool is_close_to_zero(double value) {
            return value < BINARY_PRECISION && value > -1.0 * BINARY_PRECISION;
        }

        inline bool is_close_to_one(double value) {
            return value > 1.0 - BINARY_PRECISION && value < 1.0 + BINARY_PRECISION;
        }
    }
}

#endif //QUAKE_MATH_H
