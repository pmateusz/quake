#ifndef QUAKE_ROBUST_BOX_MEAN_VAR_INDEX_MIP_MODEL_H
#define QUAKE_ROBUST_BOX_MEAN_VAR_INDEX_MIP_MODEL_H

#include <boost/date_time.hpp>

#include "base_box_mean_var_mip_model.h"

namespace quake {

    class RobustBoxMeanVarIndexMipModel : public BaseBoxMeanVarMipModel {
    public:
        RobustBoxMeanVarIndexMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

        void DecomposeTrafficIndexConstraint(const GroundStation &station);

        void DecomposeExpectationConstraint();

    private:
        GRBVar traffic_index_var_;
    };
}


#endif //QUAKE_ROBUST_BOX_MEAN_VAR_INDEX_MIP_MODEL_H
