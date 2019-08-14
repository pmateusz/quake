#ifndef QUAKE_SOC_MEAN_STD_MIP_MODEL_H
#define QUAKE_SOC_MEAN_STD_MIP_MODEL_H

#include "base_robust_mip_model.h"

namespace quake {

    class SocMeanStdMipModel : public BaseRobustMipModel {
    public:
        SocMeanStdMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

    protected:
        void Build(const boost::optional<Solution> &solution) override;
    };
}


#endif //QUAKE_SOC_MEAN_STD_MIP_MODEL_H
