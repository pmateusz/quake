#include "soc_mean_std_mip_model.h"
#include "util/math.h"

quake::SocMeanStdMipModel::SocMeanStdMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {

}

void quake::SocMeanStdMipModel::Build(const boost::optional<Solution> &solution) {
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseRobustMipModel::Build(solution);

    GRBVar probability_dual = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "probability_dual_m");

    std::vector<std::vector<GRBVar> > mean_dual
            = util::CreateVarMatrix(mip_model_, NumStations(), NumCloudCoverPeriods(), -GRB_INFINITY, GRB_INFINITY, "mean_dual_m");

    std::vector<std::vector<GRBVar> > variance_dual
            = util::CreateVarMatrix(mip_model_, NumStations(), NumCloudCoverPeriods(), 0, GRB_INFINITY, "variance_dual_m");

    GRBVar traffic_index_ub = mip_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "traffic_index_ub_m");

    {
        // configure objective
        GRBLinExpr objective_expr = probability_dual;
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = Index(station);
            for (const auto &period : CloudCover(station)) {
                const auto period_index = CloudCoverIndex(period);
                objective_expr += CloudCoverMean(station, period) * mean_dual.at(station_index).at(period_index);
                objective_expr += CloudCoverVariance(station, period) * variance_dual.at(station_index).at(period_index);
            }
        }

        mip_model_.setObjective(objective_expr);
        mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    }

    {
        // configure traffic index upper bound
        for (const auto &station : ObservableStations()) {
            mip_model_.addConstr(TransferShare(station) * traffic_index_ub <= GetMaxKeysTransferredExpr(station));
        }
    }

    {
        // configure relaxed problem
        GRBLinExpr relaxed_constraint_expr = probability_dual;
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = Index(station);
            for (const auto &period : CloudCover(station)) {
                const auto period_index = CloudCoverIndex(period);
                relaxed_constraint_expr += CloudCoverMean(station, period) * mean_dual.at(station_index).at(period_index);
                relaxed_constraint_expr += CloudCoverVariance(station, period) * variance_dual.at(station_index).at(period_index);
            }
        }
        mip_model_.addConstr(relaxed_constraint_expr >= -traffic_index_ub);
    }

    callback_ = std::make_unique<MasterCallback>(*this);
    mip_model_.setCallback(callback_.get());
}


quake::SocMeanStdMipModel::MasterCallback::MasterCallback(SocMeanStdMipModel &model)
        : remote_model_{model},
          local_model_{remote_model_.mip_environment_} {
    SetupLocalModel();
}

void quake::SocMeanStdMipModel::MasterCallback::SetupLocalModel() {
    local_model_.set(GRB_IntParam_LazyConstraints, 1);

    probability_dual_ = local_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "probability_dual_s");
    mean_dual_ = util::CreateVarMatrix(local_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(),
                                       -GRB_INFINITY, GRB_INFINITY, "mean_dual_s");
    variance_dual_ = util::CreateVarMatrix(local_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(),
                                           0, GRB_INFINITY, "variance_dual_s");
    traffic_index_ub_ = local_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "s_traffic_index_ub");

    {
        // configure objective
        GRBLinExpr objective_expr = probability_dual_;
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                objective_expr += remote_model_.CloudCoverMean(station, period) * mean_dual_.at(station_index).at(period_index);
                objective_expr += remote_model_.CloudCoverVariance(station, period) * variance_dual_.at(station_index).at(period_index);
            }
        }

        local_model_.setObjective(objective_expr);
        local_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    }

    {
        // configure main constraint
        GRBLinExpr constraint_expr = probability_dual_;
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                constraint_expr += remote_model_.CloudCoverMean(station, period) * mean_dual_.at(station_index).at(period_index);
                constraint_expr += remote_model_.CloudCoverVariance(station, period) * variance_dual_.at(station_index).at(period_index);
            }
        }

        local_model_.addConstr(constraint_expr >= -traffic_index_ub_, "master_constraint");
    }
}

void quake::SocMeanStdMipModel::MasterCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    ResetLocalModel();



    // worst case is always zero
//    {
//        // define worst case traffic index constraints which may happen if cloud cover is 1
//
//        for (const auto &station : model_.ObservableStations()) {
//            const auto station_index = model_.Index(station);
//
//            GRBLinExpr keys_transferred = 0;
//            for (const auto &period : model_.ObservableCloudCover(station)) {
//                const auto period_index = model_.CloudCoverIndex(period);
//
//                // assume worst cloud cover is always 1
//                GRBLinExpr keys_transferred_period = 0;
//            }
//        }
//    }

    bool model_updated = false;
    do {
        local_model_.optimize();
        const auto status_code = local_model_.get(GRB_IntAttr_Status);
        const auto solver_status = static_cast<util::SolverStatus >(status_code);
        CHECK_EQ(solver_status, util::SolverStatus::Optimal);

        std::stringstream msg;
        msg << "Intercept: " << std::setprecision(2) << probability_dual_.get(GRB_DoubleAttr_X) << std::endl
            << " Traffic Index: " << std::setprecision(2) << traffic_index_ub_.get(GRB_DoubleAttr_X) << std::endl;

        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            msg << station << std::endl;
            msg << "\tMean:    ";
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                msg << " " << std::setprecision(2) << mean_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
            }
            msg << std::endl << "\tVariance:";
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                msg << " " << std::setprecision(2) << variance_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
            }
            msg << std::endl;
        }

        LOG(INFO) << msg.str();
        model_updated = SolveUncertainty();
    } while (model_updated);

    LOG(INFO) << "Next Iteration";
}

double quake::SocMeanStdMipModel::MasterCallback::GetSolutionMaxKeyRate(const quake::GroundStation &station) {
    double key_rate = 0.0;
    for (const auto &interval : remote_model_.StationIntervals(station)) {
        key_rate += getSolution(interval.Var()) * remote_model_.problem_->KeyRate(station, interval.Period());
    }
    return key_rate;
}

double quake::SocMeanStdMipModel::MasterCallback::GetSolutionMaxKeyRate(const quake::GroundStation &station,
                                                                        const boost::posix_time::time_period &period) {
    double key_rate = 0.0;
    for (const auto &interval : remote_model_.GetIntervals(station, period)) {
        key_rate += getSolution(interval.Var()) * remote_model_.problem_->KeyRate(station, interval.Period());
    }
    return key_rate;
}

void quake::SocMeanStdMipModel::MasterCallback::ResetLocalModel() {
    const auto num_constraints = local_model_.get(GRB_IntAttr_NumConstrs);
    if (num_constraints > 0) {
        std::vector<GRBConstr> constraints_to_remove;
        for (auto constraint_index = 0; constraint_index < num_constraints; ++constraint_index) {
            const auto constraint = local_model_.getConstr(constraint_index);
            if (constraint.get(GRB_StringAttr_ConstrName) != "master_constraint") {
                constraints_to_remove.push_back(constraint);
            }
        }

        for (const auto &constraint : constraints_to_remove) {
            local_model_.remove(constraint);
        }
    }
    local_model_.reset(0);
    local_model_.update();

    CHECK_EQ(local_model_.get(GRB_IntAttr_NumConstrs), 1);

    // configure traffic index upper bound
    for (const auto &station : remote_model_.ObservableStations()) {
        local_model_.addConstr(remote_model_.TransferShare(station) * traffic_index_ub_ <= GetSolutionMaxKeyRate(station));
    }
}

bool quake::SocMeanStdMipModel::MasterCallback::SolveUncertainty() {
    GRBModel uncertain_model_{remote_model_.mip_environment_};

    GRBVar traffic_index = uncertain_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "un_traffic_index");

    std::vector<std::vector<GRBVar> > un_variance
            = util::CreateVarMatrix(uncertain_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(),
                                    remote_model_.CloudCoverVarianceLowerBound(), GRB_INFINITY, "un_variance");

    std::vector<std::vector<GRBVar> > un_cloud_cover;
    un_cloud_cover.resize(remote_model_.NumStations());
    for (const auto &station : remote_model_.Stations()) {
        if (station == GroundStation::None) { continue; }

        const auto station_index = remote_model_.Index(station);
        un_cloud_cover.at(station_index).reserve(remote_model_.NumCloudCoverPeriods());
        for (const auto &period : remote_model_.CloudCover(station)) {
            un_cloud_cover.at(station_index).emplace_back(uncertain_model_.addVar(remote_model_.CloudCoverLowerBound(station, period),
                                                                                  remote_model_.CloudCoverUpperBound(station, period),
                                                                                  0,
                                                                                  GRB_CONTINUOUS));
        }
    }

    {
        // configure relation between variance and cloud cover
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                auto mean_offset = un_cloud_cover.at(station_index).at(period_index) - remote_model_.CloudCoverMean(station, period);
                uncertain_model_.addQConstr(mean_offset * mean_offset <= un_variance.at(station_index).at(period_index));
            }
        }
    }

    {
        // configure objective
        GRBLinExpr objective = probability_dual_.get(GRB_DoubleAttr_X);
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);

                objective += mean_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X) * un_cloud_cover.at(station_index).at(period_index);
                objective += variance_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X) * un_variance.at(station_index).at(period_index);
            }
        }
        objective += traffic_index;

        uncertain_model_.setObjective(objective);
        uncertain_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    }

    {
        for (const auto &station : remote_model_.ObservableStations()) {
            const auto station_index = remote_model_.Index(station);

            GRBLinExpr constraint_expr = remote_model_.TransferShare(station) * traffic_index;
            for (const auto &period : remote_model_.ObservableCloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                const auto max_keys_transferred = GetSolutionMaxKeyRate(station, period);
                constraint_expr -= max_keys_transferred;
                constraint_expr += max_keys_transferred * un_cloud_cover.at(station_index).at(period_index);
            }
            uncertain_model_.addConstr(constraint_expr <= 0.0);
        }
    }

    uncertain_model_.optimize();

    const auto solver_status_code = uncertain_model_.get(GRB_IntAttr_Status);
    const auto solver_status = static_cast<util::SolverStatus >(solver_status_code);

    CHECK(solver_status == util::SolverStatus::Optimal || solver_status == util::SolverStatus::Suboptimal);
    LOG(INFO) << solver_status_code;

    const auto objective_value = uncertain_model_.get(GRB_DoubleAttr_ObjVal);
    if (objective_value < 0) {
        // rerun needed

        GRBLinExpr master_constraint_expr = probability_dual_;
        double master_constraint_value = probability_dual_.get(GRB_DoubleAttr_X);
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);

                master_constraint_expr
                        += mean_dual_.at(station_index).at(period_index) * un_cloud_cover.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
                master_constraint_expr
                        += variance_dual_.at(station_index).at(period_index) * un_variance.at(station_index).at(period_index).get(GRB_DoubleAttr_X);

                master_constraint_value
                        += mean_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X) *
                           un_cloud_cover.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
                master_constraint_value
                        += variance_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X) *
                           un_variance.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
            }
        }
        master_constraint_expr += traffic_index_ub_;
        master_constraint_value += traffic_index.get(GRB_DoubleAttr_X);
        CHECK(util::is_nearly_eq(master_constraint_value, objective_value));

        LOG(INFO) << "Adding cut: " << objective_value << " >= " << 0;
        local_model_.addConstr(master_constraint_expr >= 0);

        LOG(INFO) << "Traffic Index (for uncertain problem): " << traffic_index.get(GRB_DoubleAttr_X);
        if (traffic_index_ub_.get(GRB_DoubleAttr_X) > traffic_index.get(GRB_DoubleAttr_X)) {
            for (const auto &station : remote_model_.ObservableStations()) {
                const auto station_index = remote_model_.Index(station);

                double constraint_value = remote_model_.TransferShare(station) * traffic_index_ub_.get(GRB_DoubleAttr_X);
                GRBLinExpr constraint_expr = remote_model_.TransferShare(station) * traffic_index_ub_;
                for (const auto &period : remote_model_.ObservableCloudCover(station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    const auto max_keys_transferred = GetSolutionMaxKeyRate(station, period);
                    constraint_expr -= max_keys_transferred;
                    constraint_value -= max_keys_transferred;
                    constraint_expr += max_keys_transferred * un_cloud_cover.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
                    constraint_value += max_keys_transferred * un_cloud_cover.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
                }
                LOG(INFO) << "Traffic Index cut: " << constraint_value << " <= 0";
                local_model_.addConstr(constraint_expr <= 0.0);
            }
        }

        return true;
    }

    return false;
}
