#ifndef QUAKE_ROBUST_BOX_INDEX_MIP_MODEL_H
#define QUAKE_ROBUST_BOX_INDEX_MIP_MODEL_H

#include <vector>

#include <boost/date_time.hpp>

#include "base_robust_mip_model.h"

namespace quake {

    class RobustBoxIndexMipModel : public BaseRobustMipModel {
    public:
        RobustBoxIndexMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

    private:
        std::vector<std::vector<GRBVar> > cloud_cover_lower_bound_dual_;
        std::vector<std::vector<GRBVar> > cloud_cover_upper_bound_dual_;
    };
}


#endif //QUAKE_ROBUST_BOX_INDEX_MIP_MODEL_H
