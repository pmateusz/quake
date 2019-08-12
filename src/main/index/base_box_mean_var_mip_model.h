#ifndef QUAKE_BASEBOXMEANVARMIPMODEL_H
#define QUAKE_BASEBOXMEANVARMIPMODEL_H

#include <boost/date_time.hpp>

#include "base_robust_mip_model.h"

namespace quake {

    class BaseBoxMeanVarMipModel : public BaseRobustMipModel {
    public:
        BaseBoxMeanVarMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step);

    protected:
        class ReformulationSession {
        public:
            ReformulationSession(BaseBoxMeanVarMipModel &model, std::string prefix);

            void FillIntercept(GRBLinExpr &expression, const GroundStation &station, const boost::posix_time::time_period &time_period) const;

            void FillDualConstraint(GRBLinExpr &expression, const GroundStation &station, const boost::posix_time::time_period &time_period) const;

            std::vector<std::vector<GRBVar>> CreateCloudCoverDuals(std::string prefix);

            bool empty() const;

        private:
            std::string CloudCoverLowerBoundLabel() const;

            std::string CloudCoverUpperBoundLabel() const;

            std::string VarLowerBoundLabel() const;

            std::string VarUpperBoundLabel() const;

            BaseBoxMeanVarMipModel &model_;
            std::string prefix_;

            std::vector<std::vector<GRBVar> > cc_lb_dual_;
            std::vector<std::vector<GRBVar> > cc_ub_dual_;
            std::vector<std::vector<GRBVar> > var_lb_dual_;
            std::vector<std::vector<GRBVar> > var_ub_dual_;
        };

        friend class ReformulationSession;

        bool IsFirstCloudCoverPeriod(const GroundStation &station, const boost::posix_time::time_period &period) const;

        bool IsLastCloudCoverPeriod(const GroundStation &station, const boost::posix_time::time_period &period) const;
    };
}

#endif //QUAKE_BASEBOXMEANVARMIPMODEL_H
