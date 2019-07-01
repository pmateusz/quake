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

#ifndef QUAKE_FIXED_DISCRETISATION_SCHEME_H
#define QUAKE_FIXED_DISCRETISATION_SCHEME_H

#include <boost/date_time.hpp>

#include <glog/logging.h>
#include <ortools/base/logging.h>

#include "base_interval.h"
#include "inferred_model.h"

namespace quake {

    struct DiscretisationScheme {
        DiscretisationScheme(std::vector<BaseInterval> transfer_intervals, std::vector<BaseInterval> switch_intervals);

        std::vector<BaseInterval> TransferIntervals;
        std::vector<BaseInterval> SwitchIntervals;
    };

    class FixedDiscretisationScheme {
    public:
        FixedDiscretisationScheme(const quake::InferredModel &model, boost::posix_time::time_duration time_step);

        DiscretisationScheme Build();

    private:
        BaseInterval GetTransferInterval(std::size_t begin_index) const;

        BaseInterval GetSwitchInterval(std::size_t transfer_start_index) const;

        const quake::InferredModel &model_;
        const std::size_t num_time_;
        const std::size_t dummy_station_;

        boost::posix_time::time_duration time_step_;
    };
}


#endif //QUAKE_FIXED_DISCRETISATION_SCHEME_H
