#include "robust_box_mean_var_index_mip_model.h"

quake::RobustBoxMeanVarIndexMipModel::RobustBoxMeanVarIndexMipModel(const quake::ExtendedProblem *problem,
                                                                    boost::posix_time::time_duration interval_step)
        : BaseBoxMeanVarMipModel(problem, std::move(interval_step)) {}

void quake::RobustBoxMeanVarIndexMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // key rate should be taken from problem because it is scenario independent
    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    traffic_index_var_ = mip_model_.addVar(0, traffic_index_ub, 0, GRB_CONTINUOUS);

    for (const auto &station : ObservableStations()) {
        DecomposeTrafficIndexConstraint(station);
    }

    DecomposeExpectationConstraint();
}

void quake::RobustBoxMeanVarIndexMipModel::DecomposeTrafficIndexConstraint(const quake::GroundStation &station) {
    LOG(FATAL) << "not implemented";
}

void quake::RobustBoxMeanVarIndexMipModel::DecomposeExpectationConstraint() {
    LOG(FATAL) << "not implemented";
}
