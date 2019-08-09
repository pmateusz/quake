#ifndef QUAKE_ROBUST_BOX_MEAN_INDEX_MIP_MODEL_H
#define QUAKE_ROBUST_BOX_MEAN_INDEX_MIP_MODEL_H

#include <vector>

#include <boost/date_time.hpp>

#include "base_robust_mip_model.h"

namespace quake {

    class RobustBoxMeanIndexMipModel : public BaseRobustMipModel {
    public:
        RobustBoxMeanIndexMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step, double target_index);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

    private:
        double target_index_;
    };
}


#endif //QUAKE_ROBUST_BOX_MEAN_INDEX_MIP_MODEL_H
