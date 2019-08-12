#include "base_box_mean_var_mip_model.h"

#include <utility>

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
        : ReformulationSession(model, std::move(prefix), boost::none) {}


quake::BaseBoxMeanVarMipModel::ReformulationSession::ReformulationSession(quake::BaseBoxMeanVarMipModel &model,
                                                                          std::string prefix,
                                                                          double error_multiplier)
        : ReformulationSession(model, std::move(prefix), boost::make_optional(error_multiplier)) {}

quake::BaseBoxMeanVarMipModel::ReformulationSession::ReformulationSession(quake::BaseBoxMeanVarMipModel &model,
                                                                          std::string prefix,
                                                                          boost::optional<double> error_multiplier)
        : model_{model},
          prefix_{std::move(prefix)},
          error_multiplier_{std::move(error_multiplier)},
          cc_lb_dual_{},
          cc_ub_dual_{},
          var_lb_dual_{},
          var_ub_dual_{} {}

std::vector<std::vector<GRBVar>> quake::BaseBoxMeanVarMipModel::ReformulationSession::CreateDuals(double lower_bound,
                                                                                                  double upper_bound,
                                                                                                  const std::string &prefix) {
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
            container_row.emplace_back(model_.mip_model_.addVar(lower_bound, upper_bound, 0, GRB_CONTINUOUS, label.str()));
        }
    }

    return container;
}

static const auto NORMALIZATION_CONST = 100.0;

void quake::BaseBoxMeanVarMipModel::ReformulationSession::FillIntercept(GRBLinExpr &expression,
                                                                        const quake::GroundStation &station,
                                                                        const boost::posix_time::time_period &time_period) const {
    CHECK(!cc_ub_dual_.empty());
    CHECK(!cc_lb_dual_.empty());
    CHECK(!var_ub_dual_.empty());
    CHECK(!var_lb_dual_.empty());

    const auto station_index = model_.Index(station);
    const auto cloud_cover_index = model_.CloudCoverIndex(time_period);
    const auto cloud_cover_lower_bound = model_.CloudCoverLowerBound(station, time_period);
    const auto cloud_cover_upper_bound = model_.CloudCoverUpperBound(station, time_period);
    CHECK_LE(cloud_cover_lower_bound, cloud_cover_upper_bound);

    expression += cloud_cover_upper_bound * cc_ub_dual_.at(station_index).at(cloud_cover_index);
    expression -= cloud_cover_lower_bound * cc_lb_dual_.at(station_index).at(cloud_cover_index);

    if (!model_.IsFirstCloudCoverPeriod(station, time_period)) {
        const auto &var_model = model_.problem_->VarModel(station);
        expression -= (var_model.Intercept.Value - *error_multiplier_ * (var_model.Residual.Value + var_model.Residual.Stderr))
                      * var_ub_dual_.at(station_index).at(cloud_cover_index);
        expression += (var_model.Intercept.Value + *error_multiplier_ * (var_model.Residual.Value + var_model.Residual.Stderr))
                      * var_lb_dual_.at(station_index).at(cloud_cover_index);
    }
}

void quake::BaseBoxMeanVarMipModel::ReformulationSession::FillDualConstraint(GRBLinExpr &expression,
                                                                             const quake::GroundStation &station,
                                                                             const boost::posix_time::time_period &time_period) const {
    CHECK(!cc_ub_dual_.empty());
    CHECK(!cc_lb_dual_.empty());
    CHECK(!var_ub_dual_.empty());
    CHECK(!var_lb_dual_.empty());

    const auto station_index = model_.Index(station);
    const auto cloud_cover_index = model_.CloudCoverIndex(time_period);

    expression -= cc_ub_dual_.at(station_index).at(cloud_cover_index);
    expression += cc_lb_dual_.at(station_index).at(cloud_cover_index);

    if (!model_.IsFirstCloudCoverPeriod(station, time_period)) {
        expression += NORMALIZATION_CONST * var_ub_dual_.at(station_index).at(cloud_cover_index);
        expression -= NORMALIZATION_CONST * var_lb_dual_.at(station_index).at(cloud_cover_index);
    }

    if (!model_.IsLastCloudCoverPeriod(station, time_period)) {
        const auto next_cloud_cover_index = cloud_cover_index + 1;
        for (const auto &other_station : model_.Stations()) {
            if (other_station == GroundStation::None) {
                continue;
            }

            const auto other_station_index = model_.Index(other_station);
            const auto &other_var_model = model_.problem_->VarModel(other_station);
            expression -= NORMALIZATION_CONST
                          * var_ub_dual_.at(other_station_index).at(next_cloud_cover_index)
                          * other_var_model.Parameters.at(station).Value;
            expression += NORMALIZATION_CONST
                          * var_lb_dual_.at(other_station_index).at(next_cloud_cover_index)
                          * other_var_model.Parameters.at(station).Value;
        }
    }
}

void quake::BaseBoxMeanVarMipModel::ReformulationSession::Init() {
    cc_lb_dual_ = std::move(CreateDuals(0, GRB_INFINITY, CloudCoverLowerBoundLabel()));
    cc_ub_dual_ = std::move(CreateDuals(0, GRB_INFINITY, CloudCoverUpperBoundLabel()));
    var_lb_dual_ = std::move(CreateDuals(0, GRB_INFINITY, VarLowerBoundLabel()));
    var_ub_dual_ = std::move(CreateDuals(0, GRB_INFINITY, VarUpperBoundLabel()));
    error_multiplier_ = AdaptErrorMultiplier();
}

double quake::BaseBoxMeanVarMipModel::ReformulationSession::AdaptErrorMultiplier() const {
    LOG(INFO) << "Computing value of the error multiplier";

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

    GRBVar error_contrib = proof_model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);

    // define constraints for each station
    for (const auto &output_station : model_.Stations()) {
        if (output_station == GroundStation::None) {
            continue;
        }

        const auto output_station_index = model_.Index(output_station);
        const auto &var_model = model_.problem_->VarModel(output_station);
        LOG(INFO) << var_model.Intercept.Value << " " << var_model.Residual.Stderr;

        const auto &cloud_cover_periods = model_.CloudCover(output_station);

        for (std::size_t output_index = 1; output_index < cloud_cover_vars.at(output_station_index).size(); ++output_index) {
            auto prev_index = output_index - 1;

            // left hand-side of var model
            GRBLinExpr lower_series_expr = var_model.Intercept.Value;
            GRBLinExpr upper_series_expr = var_model.Intercept.Value;
            for (const auto &other_station: model_.Stations()) {
                if (other_station == GroundStation::None) {
                    continue;
                }

                const auto other_station_index = model_.Index(other_station);
                const auto other_station_param = var_model.Parameters.at(other_station);
                lower_series_expr += NORMALIZATION_CONST * other_station_param.Value
                                     * cloud_cover_vars.at(other_station_index).at(prev_index);
                upper_series_expr += NORMALIZATION_CONST * other_station_param.Value
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
    CHECK_EQ(solver_status, util::SolverStatus::Optimal);

    const auto error_multiplier = proof_model.get(GRB_DoubleAttr_ObjVal);
    static const auto MIN_ERROR_MULTIPLIER = 5;
    if (error_multiplier > MIN_ERROR_MULTIPLIER) {
        LOG(WARNING) << "Error multiplier " << error_multiplier << " exceeds the safety margin ("
                     << MIN_ERROR_MULTIPLIER << ")";
        return error_multiplier;
    } else {
        LOG(INFO) << "Error multiplier " << error_multiplier << " stays within the safety margin ("
                  << MIN_ERROR_MULTIPLIER << "), so using the safety margin instead";
        return MIN_ERROR_MULTIPLIER;
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