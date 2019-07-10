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

#ifndef QUAKE_ROBUST_MIP_MODEL_H
#define QUAKE_ROBUST_MIP_MODEL_H

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/date_time.hpp>

#include <gurobi_c++.h>

#include "extended_problem.h"
#include "solution.h"
#include "robust/interval_var.h"

namespace quake {

    class RobustMipModel {
    public:
        RobustMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

        boost::optional<Solution> Solve(const boost::optional<boost::posix_time::time_duration> &time_limit_opt,
                                        const boost::optional<double> &gap_opt);

    private:
        inline std::size_t Index(const GroundStation &station) const { return station_indices_.at(station); }

        robust::IntervalVar CreateInterval(std::size_t station_index, boost::posix_time::time_period period);

        ExtendedProblem const *problem_;
        boost::posix_time::time_duration interval_step_;

        std::vector<GroundStation> stations_;
        std::unordered_map<GroundStation, std::size_t> station_indices_;

        std::vector<std::vector<robust::IntervalVar> > intervals_;

        GRBEnv mip_environment_;
        GRBModel mip_model_;
    };
}


#endif //QUAKE_ROBUST_MIP_MODEL_H
