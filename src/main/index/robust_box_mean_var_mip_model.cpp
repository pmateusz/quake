#include "robust_box_mean_var_mip_model.h"

quake::RobustBoxMeanVarMipModel::RobustBoxMeanVarMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseBoxMeanVarMipModel(problem, std::move(interval_step)) {}

void quake::RobustBoxMeanVarMipModel::Build(const boost::optional<Solution> &solution) {
    mip_model_.set(GRB_IntParam_InfUnbdInfo, 1.0);

    BaseIntervalMipModel::Build(solution);

    // key rate should be taken from problem because it is scenario independent
    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    traffic_index_var_ = mip_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);

    for (const auto &station : ObservableStations()) {
        DecomposeTrafficIndexConstraint(station);
    }

    GRBLinExpr objective = traffic_index_var_;
    mip_model_.setObjective(objective);
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);

//    DecomposeExpectationConstraint();
}

void quake::RobustBoxMeanVarMipModel::DecomposeTrafficIndexConstraint(const quake::GroundStation &master_station) {
    const auto master_station_index = Index(master_station);
    std::stringstream session_label;
    session_label << "traffic_index_" << master_station_index;
    ReformulationSession session = ReformulationSession(*this, session_label.str());

    GRBLinExpr traffic_index_expr = TransferShare(master_station) * traffic_index_var_;
    for (const auto &local_station : Stations()) {
        if (local_station == GroundStation::None) {
            continue;
        }

        for (const auto &cloud_cover_period: CloudCover(local_station)) {
            GRBLinExpr local_dual_expr = 0;

            if (local_station == master_station) {
                const auto transfer_intervals = GetIntervals(local_station, cloud_cover_period);
                if (!transfer_intervals.empty()) {
                    // keys transferred in the cloud cover period
                    GRBLinExpr keys_transferred = 0;
//                    for (const auto &interval : GetIntervals(local_station, cloud_cover_period)) {
//                        keys_transferred += interval.Var() * problem_->KeyRate(local_station, interval.Period());
//                    }

                    traffic_index_expr -= keys_transferred;
                    local_dual_expr += keys_transferred;
                }
            }

            session.FillIntercept(traffic_index_expr, local_station, cloud_cover_period);
            session.FillDualConstraint(local_dual_expr, local_station, cloud_cover_period);
            mip_model_.addConstr(local_dual_expr == 0);
        }
    }

    mip_model_.addConstr(traffic_index_expr <= 0);
}

void quake::RobustBoxMeanVarMipModel::DecomposeExpectationConstraint() {
    ReformulationSession session = ReformulationSession(*this, "expectation");

    auto mean_dual = session.CreateCloudCoverDuals("mean_dual");

    GRBVar mean_intercept_dual = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);
    GRBLinExpr objective = mean_intercept_dual;
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);
        for (const auto &cloud_cover_period : ObservableCloudCover(station)) {
            const auto cloud_cover_index = CloudCoverIndex(cloud_cover_period);

            objective += CloudCoverMean(station, cloud_cover_period) * mean_dual.at(station_index).at(cloud_cover_index);
        }
    }

    GRBLinExpr intercept_lower_bound_expr = -traffic_index_var_;
    for (const auto &station : Stations()) {
        if (station == GroundStation::None) {
            continue;
        }

        const auto station_index = Index(station);
        for (const auto &cloud_cover_period : CloudCover(station)) {
            const auto cloud_cover_index = CloudCoverIndex(cloud_cover_period);

            session.FillIntercept(intercept_lower_bound_expr, station, cloud_cover_period);

            GRBLinExpr dual_constraint_expr = -mean_dual.at(station_index).at(cloud_cover_index);
            session.FillDualConstraint(dual_constraint_expr, station, cloud_cover_period);
            mip_model_.addConstr(dual_constraint_expr == 0);
        }
    }

    mip_model_.addConstr(mean_intercept_dual >= intercept_lower_bound_expr);
    mip_model_.setObjective(objective);
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
}