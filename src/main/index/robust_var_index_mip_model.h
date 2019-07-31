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

#ifndef QUAKE_ROBUST_VAR_INDEX_MIP_MODEL_H
#define QUAKE_ROBUST_VAR_INDEX_MIP_MODEL_H

#include <gurobi_c++.h>

#include "extended_problem.h"
#include "base_interval_mip_model.h"

namespace quake {

    class RobustVarIndexMipModel : public BaseIntervalMipModel {
    public:
        RobustVarIndexMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step, double target_index);

    private:
    protected:
        void Build(const boost::optional<Solution> &solution) override;

        std::size_t GetCloudCoverIndex(const boost::posix_time::time_period &period) const;

    private:

        class RobustVarIndexMipCallback : public GRBCallback {
        public:
            explicit RobustVarIndexMipCallback(RobustVarIndexMipModel &model);

        protected:
            void callback() override;

        private:
            std::vector<std::vector<GRBVar> > CreateCloudCoverVariables(GRBModel &model);

            RobustVarIndexMipModel &model_;
            double traffic_index_upper_bound_;

            void callback_combined();

            void callback_slack();

            void callback_best_at_zero();
        };

        friend class RobustVarIndexMipCallback;

        double target_index_;

        std::vector<boost::posix_time::time_period> cloud_cover_index_;

        // variable r in the dual reformulation
        GRBVar dual_intercept_;

        // variables s in the dual reformulation
        std::vector<std::vector<std::size_t> > used_cloud_cover_indices_;
        std::vector<std::vector<GRBVar>> dual_cloud_cover_by_station_;

        std::unique_ptr<RobustVarIndexMipCallback> callback_;
    };
}


#endif //QUAKE_ROBUST_VAR_INDEX_MIP_MODEL_H
