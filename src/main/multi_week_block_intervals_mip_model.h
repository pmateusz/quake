#ifndef QUAKE_MULTI_WEEK_BLOCK_INTERVALS_MIP_MODEL_H
#define QUAKE_MULTI_WEEK_BLOCK_INTERVALS_MIP_MODEL_H

#include "index/base_interval_mip_model.h"

namespace quake {

    class MultiWeekBlockIntervalsMipModel : public BaseIntervalMipModel {
    public:
        MultiWeekBlockIntervalsMipModel(ExtendedProblem const *problem, Forecast forecast, boost::posix_time::time_duration time_step);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

        void ReportResults(util::SolverStatus solver_status) override;

    private:
        GRBVar lambda_;
        GRBVar total_keys_;
    };
}


#endif //QUAKE_MULTI_WEEK_BLOCK_INTERVALS_MIP_MODEL_H
