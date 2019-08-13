#ifndef QUAKE_BASE_ROBUST_MIP_MODEL_H
#define QUAKE_BASE_ROBUST_MIP_MODEL_H

#include <vector>

#include <boost/date_time.hpp>

#include "base_interval_mip_model.h"

namespace quake {
    class BaseRobustMipModel : public BaseIntervalMipModel {
    public:
        BaseRobustMipModel(ExtendedProblem const *problem, boost::posix_time::time_duration interval_step, std::vector<Forecast> forecasts);

    protected:
        void CreateCloudCoverDuals(std::vector<std::vector<GRBVar>> &container); // TODO: move to utils

        std::size_t CloudCoverIndex(const boost::posix_time::time_period &period) const;

        double CloudCoverMean(const GroundStation &station, const boost::posix_time::time_period &period) const;

        double CloudCoverLowerBound(const GroundStation &station, const boost::posix_time::time_period &period) const;

        double CloudCoverUpperBound(const GroundStation &station, const boost::posix_time::time_period &period) const;

        const std::vector<boost::posix_time::time_period> &CloudCover(const GroundStation &station) const;

        const std::vector<boost::posix_time::time_period> &ObservableCloudCover(const GroundStation &station) const;

        std::vector<std::vector<GRBVar>> CreateDuals(double lower_bound, double upper_bound, const std::string &prefix) const;

    private:
        double GetCloudCoverValue(const std::vector<double> &container, const boost::posix_time::time_period &period) const;

        std::vector<boost::posix_time::time_period> cloud_cover_periods_;
        std::vector<std::vector<boost::posix_time::time_period>> observable_cloud_cover_periods_;
    };
}


#endif //QUAKE_BASE_ROBUST_MIP_MODEL_H
