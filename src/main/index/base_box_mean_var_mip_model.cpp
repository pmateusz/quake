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

bool quake::BaseBoxMeanVarMipModel::ReformulationSession::empty() const {
    GRBModel proof_model{model_.mip_environment_};

    // define cloud cover variables
    std::vector<std::vector<GRBVar>> cloud_cover_vars;
    cloud_cover_vars.resize(model_.Stations().size());

    for (const auto &station : model_.Stations()) {
        if (station == GroundStation::None) {
            continue;
        }

        const auto station_index = model_.Index(station);
        auto &row = cloud_cover_vars.at(station_index);
        const auto &cloud_cover_periods = model_.CloudCover(station);
        row.reserve(cloud_cover_periods.size());

        for (const auto &period : cloud_cover_periods) {
            row.emplace_back(proof_model.addVar(model_.CloudCoverLowerBound(station, period),
                                                model_.CloudCoverUpperBound(station, period),
                                                0,
                                                GRB_CONTINUOUS));
        }
    }

    static const auto NORMALIZATION_CONST = 100.0;
    GRBVar error_contrib = proof_model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);

    // define constraints for each station
    for (const auto &output_station : model_.Stations()) {
        if (output_station == GroundStation::None) {
            continue;
        }

        const auto output_station_index = model_.Index(output_station);
        const auto &var_model = model_.problem_->VarModel(output_station);
        const auto &cloud_cover_periods = model_.CloudCover(output_station);

        for (std::size_t output_index = 1; output_index < cloud_cover_vars.at(output_station_index).size(); ++output_index) {
            auto prev_index = output_index - 1;

            // left hand-side of var model
            GRBLinExpr lower_series_expr = var_model.Intercept.Value; // - error_contrib * var_model.Intercept.Stderr;
            GRBLinExpr upper_series_expr = var_model.Intercept.Value; // + error_contrib * var_model.Intercept.Stderr;
            for (const auto &other_station: model_.Stations()) {
                if (other_station == GroundStation::None) {
                    continue;
                }

                const auto other_station_index = model_.Index(other_station);
                const auto other_station_param = var_model.Parameters.at(other_station);
                lower_series_expr +=
                        NORMALIZATION_CONST * (other_station_param.Value /*- error_contrib * other_station_param.Stderr*/)
                        * cloud_cover_vars.at(other_station_index).at(prev_index);
                upper_series_expr +=
                        NORMALIZATION_CONST * (other_station_param.Value /*+ error_contrib * other_station_param.Stderr*/)
                        * cloud_cover_vars.at(other_station_index).at(prev_index);
            }

            proof_model.addConstr(lower_series_expr - error_contrib * (var_model.Residual.Value + var_model.Residual.Stderr)
                                   <= NORMALIZATION_CONST * cloud_cover_vars.at(output_station_index).at(output_index));
            proof_model.addConstr(upper_series_expr + error_contrib * (var_model.Residual.Value + var_model.Residual.Stderr)
                                   >= NORMALIZATION_CONST * cloud_cover_vars.at(output_station_index).at(output_index));
        }
    }

    GRBLinExpr obj = error_contrib;
    proof_model.setObjective(obj);
    proof_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    proof_model.optimize();
    const auto status_code = proof_model.get(GRB_IntAttr_Status);
    const auto solver_status = static_cast<util::SolverStatus>(status_code);
    return solver_status == util::SolverStatus::Optimal;
}
