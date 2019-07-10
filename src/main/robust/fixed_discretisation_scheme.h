//
// Copyright 2019 Mateusz Polnik
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


#ifndef QUAKE_FIXED_DISCRETISATION_SCHEME_H
#define QUAKE_FIXED_DISCRETISATION_SCHEME_H

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include "extended_problem.h"

namespace quake {

    namespace robust {

        struct DiscretisationScheme {
            DiscretisationScheme(std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period>> observation_intervals,
                                 std::vector<boost::posix_time::time_period> switch_intervals);

            std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period>> ObservationIntervals;
            std::vector<boost::posix_time::time_period> SwitchIntervals;
        };

        class FixedDiscretisationScheme {
        public:
            FixedDiscretisationScheme(const ExtendedProblem &problem, boost::posix_time::time_duration time_step);

            DiscretisationScheme Build() const;

        private:
            std::vector<boost::posix_time::time_period> generate_observation_intervals(const boost::posix_time::time_period &period,
                                                                                       const boost::posix_time::time_duration &time_step) const;
            
            const ExtendedProblem &problem_;
            boost::posix_time::time_duration time_step_;
        };
    }
}


#endif //QUAKE_FIXED_DISCRETISATION_SCHEME_H
