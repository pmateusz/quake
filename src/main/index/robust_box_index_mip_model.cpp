#include "robust_box_index_mip_model.h"

quake::RobustBoxIndexMipModel::RobustBoxIndexMipModel(const quake::ExtendedProblem *problem,
                                                      boost::posix_time::time_duration interval_step,
                                                      double target_index)
        : BaseIntervalMipModel(problem,
                               std::move(interval_step),
                               std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}),
          target_index_{target_index} {}

void quake::RobustBoxIndexMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    auto traffic_index_var = mip_model_.addVar(0, traffic_index_ub, 0, GRB_CONTINUOUS);

    GRBLinExpr objective = -traffic_index_var;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mip_model_.setObjective(objective);

    // TODO: create dual variable for each uncertain cloud cover
    // TODO: get index of uncertain cloud cover for each interval
}
