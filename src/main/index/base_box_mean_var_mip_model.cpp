#include "base_box_mean_var_mip_model.h"

quake::BaseBoxMeanVarMipModel::BaseBoxMeanVarMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {}

bool quake::BaseBoxMeanVarMipModel::IsFirstCloudCoverPeriod(const quake::GroundStation &station,
                                                            const boost::posix_time::time_period &period) const {
    const auto &cloud_cover_periods = CloudCover(station);
    if (cloud_cover_periods.empty()) {
        return false;
    }
    return cloud_cover_periods.at(0) == period;
}

bool quake::BaseBoxMeanVarMipModel::IsLastCloudCoverPeriod(const quake::GroundStation &station,
                                                           const boost::posix_time::time_period &period) const {
    const auto &cloud_cover_periods = CloudCover(station);
    if (cloud_cover_periods.empty()) {
        return false;
    }
    const auto index = cloud_cover_periods.size() - 1;
    return cloud_cover_periods.at(index) == period;
}

quake::BaseBoxMeanVarMipModel::ReformulationSession::ReformulationSession(quake::BaseBoxMeanVarMipModel &model, std::string prefix)
        : model_{model},
          prefix_{std::move(prefix)},
          cc_lb_dual_{CreateCloudCoverDuals(CloudCoverLowerBoundLabel())},
          cc_ub_dual_{CreateCloudCoverDuals(CloudCoverUpperBoundLabel())},
          var_lb_dual_{CreateCloudCoverDuals(VarLowerBoundLabel())},
          var_ub_dual_{CreateCloudCoverDuals(VarUpperBoundLabel())} {}

std::vector<std::vector<GRBVar>> quake::BaseBoxMeanVarMipModel::ReformulationSession::CreateCloudCoverDuals(std::string prefix) {
    std::vector<std::vector<GRBVar>> container;

    container.resize(model_.Stations().size());
    for (const auto &station : model_.Stations()) {
        const auto station_index = model_.Index(station);

        auto &container_row = container.at(station_index);
        const auto &cloud_cover_periods = model_.CloudCover(station);
        container_row.reserve(cloud_cover_periods.size());
        for (const auto &period : cloud_cover_periods) {
            const auto cloud_cover_index = model_.CloudCoverIndex(period);

            std::stringstream label;
            label << prefix << "_[" << station_index << "][" << cloud_cover_index << "]";
            container_row.emplace_back(model_.mip_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, label.str()));
        }
    }

    return container;
}

void quake::BaseBoxMeanVarMipModel::ReformulationSession::FillIntercept(GRBLinExpr &expression,
                                                                        const quake::GroundStation &station,
                                                                        const boost::posix_time::time_period &time_period) const {
    const auto station_index = model_.Index(station);
    const auto cloud_cover_index = model_.CloudCoverIndex(time_period);
    const auto cloud_cover_lower_bound = model_.CloudCoverLowerBound(station, time_period);
    const auto cloud_cover_upper_bound = model_.CloudCoverUpperBound(station, time_period);
    CHECK_LE(cloud_cover_lower_bound, cloud_cover_upper_bound);

    expression += cloud_cover_upper_bound * cc_ub_dual_.at(station_index).at(cloud_cover_index);
    expression -= cloud_cover_lower_bound * cc_lb_dual_.at(station_index).at(cloud_cover_index);

    if (!model_.IsFirstCloudCoverPeriod(station, time_period)) {
        const auto &var_model = model_.problem_->VarModel(station);
        expression -= (var_model.Intercept.Value - var_model.Residual.Value) * var_ub_dual_.at(station_index).at(cloud_cover_index);
        expression += (var_model.Intercept.Value + var_model.Residual.Value) * var_lb_dual_.at(station_index).at(cloud_cover_index);
    }
}

void quake::BaseBoxMeanVarMipModel::ReformulationSession::FillDualConstraint(GRBLinExpr &expression,
                                                                             const quake::GroundStation &station,
                                                                             const boost::posix_time::time_period &time_period) const {
    const auto station_index = model_.Index(station);
    const auto cloud_cover_index = model_.CloudCoverIndex(time_period);

    expression -= cc_ub_dual_.at(station_index).at(cloud_cover_index);
    expression += cc_lb_dual_.at(station_index).at(cloud_cover_index);

    if (!model_.IsFirstCloudCoverPeriod(station, time_period)) {
        expression += var_ub_dual_.at(station_index).at(cloud_cover_index);
        expression -= var_lb_dual_.at(station_index).at(cloud_cover_index);
    }

    if (!model_.IsLastCloudCoverPeriod(station, time_period)) {
        const auto next_cloud_cover_index = cloud_cover_index + 1;
        for (const auto &other_station : model_.Stations()) {
            if (other_station == GroundStation::None) {
                continue;
            }

            const auto other_station_index = model_.Index(other_station);
            const auto &other_var_model = model_.problem_->VarModel(other_station);
            expression -= var_ub_dual_.at(other_station_index).at(next_cloud_cover_index) * other_var_model.Parameters.at(station).Value;
            expression += var_lb_dual_.at(other_station_index).at(next_cloud_cover_index) * other_var_model.Parameters.at(station).Value;
        }
    }
}

std::string make_label(const std::string &prefix, const std::string &suffix) {
    std::stringstream label;
    label << prefix << "_" << suffix;
    return label.str();
}

std::string quake::BaseBoxMeanVarMipModel::ReformulationSession::CloudCoverLowerBoundLabel() const {
    return make_label(prefix_, "cc_lb_dual");
}

std::string quake::BaseBoxMeanVarMipModel::ReformulationSession::CloudCoverUpperBoundLabel() const {
    return make_label(prefix_, "cc_ub_dual");
}

std::string quake::BaseBoxMeanVarMipModel::ReformulationSession::VarLowerBoundLabel() const {
    return make_label(prefix_, "var_lb_dual");
}

std::string quake::BaseBoxMeanVarMipModel::ReformulationSession::VarUpperBoundLabel() const {
    return make_label(prefix_, "var_ub_dual");
}
