#ifndef QUAKE_ROBUST_BOX_INDEX_MIP_MODEL_H
#define QUAKE_ROBUST_BOX_INDEX_MIP_MODEL_H

#include "base_interval_mip_model.h"

// TODO: create base class with dual variables of cloud cover
// TODO: find dual variable for an interval
// TODO: there can be several dual variables for each cloud cover so need to operate on some indermediary index
// TODO: given interval obtain cloud cover period
// TODO: given cloud cover period and station know lower and upper bound of cloud cover and mean
// TODO: given cloud cover period and station obtain all intervals

namespace quake {

    class RobustBoxIndexMipModel : public BaseIntervalMipModel {
    public:
        RobustBoxIndexMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step, double target_index);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

    private:
        double target_index_;
    };
}


#endif //QUAKE_ROBUST_BOX_INDEX_MIP_MODEL_H
