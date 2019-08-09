#include "robust_box_mean_mip_model.h"

#include <unordered_set>

quake::RobustBoxMeanMipModel::RobustBoxMeanMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {}

void quake::RobustBoxMeanMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // key rate should be taken from problem because it is scenario independent
    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    auto traffic_index_var = mip_model_.addVar(0, traffic_index_ub, 0, GRB_CONTINUOUS);

    {
        // build traffic index constraints for each ground station
        std::vector<std::vector<GRBVar> > traffic_cc_lower_bound_dual;
        std::vector<std::vector<GRBVar> > traffic_cc_upper_bound_dual;
        CreateCloudCoverDuals(traffic_cc_lower_bound_dual);
        CreateCloudCoverDuals(traffic_cc_upper_bound_dual);

        for (const auto &station : ObservableStations()) {
            const auto station_index = Index(station);

            // left hand-side of the constraint on the traffic index
            GRBLinExpr traffic_index_expr = TransferShare(station) * traffic_index_var;
            for (const auto &cloud_cover_period : ObservableCloudCover(station)) {
                const auto dual_index = CloudCoverIndex(cloud_cover_period);
                const auto &cloud_cover_lower_bound_var = traffic_cc_lower_bound_dual.at(station_index).at(dual_index);
                const auto &cloud_cover_upper_bound_var = traffic_cc_upper_bound_dual.at(station_index).at(dual_index);

                // keys transferred in the cloud cover period
                GRBLinExpr keys_transferred = 0;
                for (const auto &interval : GetIntervals(station, cloud_cover_period)) {
                    keys_transferred += interval.Var() * problem_->KeyRate(station, interval.Period());
                }

                // equality constraint for dual cloud cover variable
                mip_model_.addConstr(keys_transferred - cloud_cover_upper_bound_var + cloud_cover_lower_bound_var == 0);

                const auto cloud_cover_lower_bound = CloudCoverLowerBound(station, cloud_cover_period);
                const auto cloud_cover_upper_bound = CloudCoverUpperBound(station, cloud_cover_period);
                CHECK_LE(cloud_cover_lower_bound, cloud_cover_upper_bound);

                traffic_index_expr += -keys_transferred
                                      + cloud_cover_upper_bound * cloud_cover_upper_bound_var
                                      - cloud_cover_lower_bound * cloud_cover_lower_bound_var;
            }

            // traffic index constraint for station
            mip_model_.addConstr(traffic_index_expr <= 0);
        }
    }

    {
        // build mean constraint and objective
        std::vector<std::vector<GRBVar> > mean_cc_lower_bound_dual;
        std::vector<std::vector<GRBVar> > mean_cc_upper_bound_dual;
        std::vector<std::vector<GRBVar> > mean_cc_dual;

        CreateCloudCoverDuals(mean_cc_lower_bound_dual);
        CreateCloudCoverDuals(mean_cc_upper_bound_dual);
        CreateCloudCoverDuals(mean_cc_dual);

        GRBVar mean_cc_intercept_dual = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);

        GRBLinExpr objective = mean_cc_intercept_dual;

        GRBLinExpr intercept_lower_bound_expr = -traffic_index_var;
        for (const auto &station : ObservableStations()) {
            const auto station_index = Index(station);
            for (const auto &cloud_cover_period : ObservableCloudCover(station)) {
                const auto cloud_cover_index = CloudCoverIndex(cloud_cover_period);

                const auto &mean_dual_var = mean_cc_dual.at(station_index).at(cloud_cover_index);
                const auto &lower_bound_dual_var = mean_cc_lower_bound_dual.at(station_index).at(cloud_cover_index);
                const auto &upper_bound_dual_var = mean_cc_upper_bound_dual.at(station_index).at(cloud_cover_index);
                const auto mean_value = CloudCoverMean(station, cloud_cover_period);
                const auto lower_bound_value = CloudCoverLowerBound(station, cloud_cover_period);
                const auto upper_bound_value = CloudCoverUpperBound(station, cloud_cover_period);
                CHECK_LE(lower_bound_value, upper_bound_value);

                objective += mean_value * mean_dual_var;
                intercept_lower_bound_expr += upper_bound_value * upper_bound_dual_var - lower_bound_value * lower_bound_dual_var;

                mip_model_.addConstr(-upper_bound_dual_var + lower_bound_dual_var - mean_dual_var == 0);
            }
        }

        mip_model_.addConstr(mean_cc_intercept_dual >= intercept_lower_bound_expr);

        mip_model_.setObjective(objective);
        mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    }
}