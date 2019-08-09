#ifndef QUAKE_ROBUST_BOX_MIP_MODEL_H
#define QUAKE_ROBUST_BOX_MIP_MODEL_H

#include <vector>

#include "base_robust_mip_model.h"

namespace quake {

    class RobustBoxMipModel : public BaseRobustMipModel {
    public:
        RobustBoxMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

    protected:
        void Build(const boost::optional<Solution> &solution) override;
    };
}


#endif //QUAKE_ROBUST_BOX_MIP_MODEL_H
