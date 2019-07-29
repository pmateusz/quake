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

#ifndef QUAKE_ROBUST_INDEX_MIP_MODEL_H
#define QUAKE_ROBUST_INDEX_MIP_MODEL_H

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/date_time.hpp>

#include <gurobi_c++.h>

#include "extended_problem.h"
#include "base_interval_mip_model.h"
#include "solution.h"
#include "interval_var.h"

namespace quake {

    class RobustIndexMipModel : public BaseIntervalMipModel {
    public:
        RobustIndexMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step, double target_index);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

        void AppendMetadata(Metadata &metadata) override;

        void ReportResults(util::SolverStatus solver_status) override;

        std::size_t GetCloudCoverIndex(const boost::posix_time::time_period &period) const;

    private:
        class RobustIndexMipCallback : public GRBCallback {
        public:
            explicit RobustIndexMipCallback(RobustIndexMipModel &model);

        protected:
            void callback() override;

            std::vector<double> SolveRiskinessIndexConstraint(const GroundStation &station);

            std::vector<double> SolveKeysTransferredConstraint(const GroundStation &station);

        private:
            std::vector<GRBVar> CreateCloudCoverVariables(const GroundStation &station, GRBModel &model) const;

            RobustIndexMipModel &model_;
        };

        friend class RobustIndexMipCallback;

        double target_index_;

        GRBVar riskiness_index_;

        std::vector<boost::posix_time::time_period> cloud_cover_index_;
        std::vector<std::vector<GRBVar>> dual_cloud_cover_;
        std::vector<GRBVar> dual_station_;

        std::unique_ptr<RobustIndexMipCallback> callback_;
    };
}


#endif //QUAKE_ROBUST_INDEX_MIP_MODEL_H
