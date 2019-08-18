#ifndef QUAKE_SOC_MEAN_STD_MIP_MODEL_H
#define QUAKE_SOC_MEAN_STD_MIP_MODEL_H

#include <memory>
#include <gurobi_c++.h>

#include "base_robust_mip_model.h"

namespace quake {

    class SocMeanStdMipModel : public BaseRobustMipModel {
    public:
        SocMeanStdMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

        class MasterCallback;

        class MasterCallback : public GRBCallback {
        public:
            explicit MasterCallback(SocMeanStdMipModel &model);

            double GetSolutionMaxKeyRate(const GroundStation &station);

            double GetSolutionMaxKeyRate(const GroundStation &station, const boost::posix_time::time_period &period);

        protected:
            void callback() override;

        private:
            void SetupLocalModel();

            void ResetLocalModel();

            void SolveUncertainty();

            SocMeanStdMipModel &remote_model_;

            GRBModel local_model_;

            GRBVar probability_dual_;
            std::vector<std::vector<GRBVar> > mean_dual_;
            std::vector<std::vector<GRBVar> > variance_dual_;
        };

        friend class MasterCallback;

    protected:
        void Build(const boost::optional<Solution> &solution) override;

    private:
        GRBVar probability_dual_;
        std::vector<std::vector<GRBVar> > mean_dual_;
        std::vector<std::vector<GRBVar> > variance_dual_;

        std::unique_ptr<MasterCallback> callback_;
    };
}


#endif //QUAKE_SOC_MEAN_STD_MIP_MODEL_H
