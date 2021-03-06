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

#ifndef QUAKE_PROBLEM_GENERATOR_H
#define QUAKE_PROBLEM_GENERATOR_H

#include <vector>
#include <unordered_map>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include "ground_station.h"
#include "extended_problem.h"

namespace quake {

    class GroundStation;

    class Problem;

    class ExtendedProblem;

    class ProblemGenerator {
    public:
        static std::unordered_map<GroundStation, double>
        GetLinearDistributionCoefficients(const std::vector<GroundStation> &ground_stations);

        Problem Create(std::vector<GroundStation> ground_stations,
                       boost::posix_time::ptime initial_epoch,
                       boost::posix_time::time_period time_period) const;

        ExtendedProblem CreateExtendedProblem(const std::vector<GroundStation> &ground_stations,
                                              double altitude,
                                              double inclination,
                                              double raan,
                                              boost::posix_time::ptime initial_epoch,
                                              boost::posix_time::time_period time_period) const;
    };
}


#endif //QUAKE_PROBLEM_GENERATOR_H
