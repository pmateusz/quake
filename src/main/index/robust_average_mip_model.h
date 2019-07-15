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


#ifndef QUAKE_ROBUST_AVERAGE_MIP_MODEL_H
#define QUAKE_ROBUST_AVERAGE_MIP_MODEL_H

#include <vector>
#include <memory>
#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/optional.hpp>

#include <gurobi_c++.h>

#include "ground_station.h"
#include "solution.h"
#include "extended_problem.h"
#include "base_interval_mip_model.h"

namespace quake {

    class RobustAverageMipModel : public BaseIntervalMipModel {
    public:
        RobustAverageMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

        void ReportResults(util::SolverStatus solver_status) override;

        std::size_t GetCloudCoverIndex(const boost::posix_time::time_period &period) const;

    private:
        class RobustAverageMipCallback : public GRBCallback {
        public:
            explicit RobustAverageMipCallback(RobustAverageMipModel &model);

        protected:
            void callback() override;

            std::vector<double> SolveKeysTransferredConstraint(const GroundStation &station);

        private:
            std::vector<GRBVar> CreateCloudCoverVariables(const GroundStation &station, GRBModel &model) const;

            RobustAverageMipModel &model_;
        };

        friend class RobustAverageMipCallback;

        std::vector<boost::posix_time::time_period> cloud_cover_index_;
        std::vector<std::vector<GRBVar>> dual_cloud_cover_;
        std::vector<GRBVar> dual_station_;

        std::unique_ptr<RobustAverageMipCallback> callback_;
    };
}


#endif //QUAKE_ROBUST_AVERAGE_MIP_MODEL_H
