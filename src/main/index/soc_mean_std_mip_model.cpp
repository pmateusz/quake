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

    probability_dual_ = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "probability_dual_m");
    mean_dual_ = util::CreateVarMatrix(mip_model_, NumStations(), NumCloudCoverPeriods(), -GRB_INFINITY, GRB_INFINITY, "mean_dual_m");
    variance_dual_ = util::CreateVarMatrix(mip_model_, NumStations(), NumCloudCoverPeriods(), 0, GRB_INFINITY, "variance_dual_m");

    {
        // configure objective
        GRBLinExpr objective_expr = probability_dual_;
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = Index(station);
            for (const auto &period : CloudCover(station)) {
                const auto period_index = CloudCoverIndex(period);
                objective_expr += CloudCoverMean(station, period) * mean_dual_.at(station_index).at(period_index);
                objective_expr += CloudCoverVariance(station, period) * variance_dual_.at(station_index).at(period_index);
            }
        }

        mip_model_.setObjective(objective_expr);
        mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    }

    {
        // configure relaxed problem
        GRBLinExpr relaxed_constraint_expr = probability_dual_;
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = Index(station);
            for (const auto &period : CloudCover(station)) {
                const auto period_index = CloudCoverIndex(period);
                relaxed_constraint_expr += CloudCoverMean(station, period) * mean_dual_.at(station_index).at(period_index);
                relaxed_constraint_expr += CloudCoverVariance(station, period) * variance_dual_.at(station_index).at(period_index);
            }
        }

        // configure traffic index upper bound
        for (const auto &station : ObservableStations()) {
            mip_model_.addConstr(relaxed_constraint_expr >= -GetMaxKeysTransferredExpr(station) / TransferShare(station));
        }
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
    local_model_.set(GRB_IntParam_Presolve, 0);
    local_model_.set(GRB_IntParam_OutputFlag, 0);

    probability_dual_ = local_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "probability_dual_s");
    mean_dual_ = util::CreateVarMatrix(local_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(),
                                       -GRB_INFINITY, GRB_INFINITY, "mean_dual_s");
    variance_dual_ = util::CreateVarMatrix(local_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(),
                                           0, GRB_INFINITY, "variance_dual_s");

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
}

void quake::SocMeanStdMipModel::MasterCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    ResetLocalModel();

    auto last_num_constraints = 0;
    while (last_num_constraints != local_model_.get(GRB_IntAttr_NumConstrs)) {
        last_num_constraints = local_model_.get(GRB_IntAttr_NumConstrs);

        local_model_.optimize();

//        std::stringstream msg;
//        msg << "Intercept: " << std::setprecision(2) << probability_dual_.get(GRB_DoubleAttr_X) << std::endl;
//        for (const auto &station : remote_model_.Stations()) {
//            if (station == GroundStation::None) { continue; }
//
//            const auto station_index = remote_model_.Index(station);
//            msg << station << std::endl;
//            msg << "\tMean:    ";
//            for (const auto &period : remote_model_.CloudCover(station)) {
//                const auto period_index = remote_model_.CloudCoverIndex(period);
//                msg << " " << std::setprecision(2) << mean_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
//            }
//            msg << std::endl << "\tVariance:";
//            for (const auto &period : remote_model_.CloudCover(station)) {
//                const auto period_index = remote_model_.CloudCoverIndex(period);
//                msg << " " << std::setprecision(2) << variance_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
//            }
//            msg << std::endl;
//        }
//        LOG(INFO) << msg.str();

        const auto status_code = local_model_.get(GRB_IntAttr_Status);
        const auto solver_status = static_cast<util::SolverStatus >(status_code);
        CHECK_EQ(solver_status, util::SolverStatus::Optimal);
        SolveUncertainty();

        local_model_.update();
    }

    auto master_objective_value = getSolution(remote_model_.probability_dual_);
    {
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                master_objective_value +=
                        remote_model_.CloudCoverMean(station, period) *
                        getSolution(remote_model_.mean_dual_.at(station_index).at(period_index));
                master_objective_value +=
                        remote_model_.CloudCoverVariance(station, period) *
                        getSolution(remote_model_.variance_dual_.at(station_index).at(period_index));
            }
        }
    }

    auto sub_problem_objective_value = probability_dual_.get(GRB_DoubleAttr_X);
    {
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                sub_problem_objective_value +=
                        remote_model_.CloudCoverMean(station, period) * mean_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
                sub_problem_objective_value +=
                        remote_model_.CloudCoverVariance(station, period) * variance_dual_.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
            }
        }
    }

    // TODO: this cut is wrong
    CHECK(util::is_nearly_eq(sub_problem_objective_value, local_model_.get(GRB_DoubleAttr_ObjVal)));
    if (util::is_surely_lt(master_objective_value, sub_problem_objective_value)) {
        GRBLinExpr master_objective_expr = remote_model_.probability_dual_;
        for (const auto &station : remote_model_.Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = remote_model_.Index(station);
            for (const auto &period : remote_model_.CloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                master_objective_expr +=
                        remote_model_.CloudCoverMean(station, period) * remote_model_.mean_dual_.at(station_index).at(period_index);
                master_objective_expr +=
                        remote_model_.CloudCoverVariance(station, period) * remote_model_.variance_dual_.at(station_index).at(period_index);
            }
        }
        addLazy(master_objective_expr >= local_model_.get(GRB_DoubleAttr_ObjVal));
        VLOG(1) << "Master Lazy Constraint: " << master_objective_value << " >= " << local_model_.get(GRB_DoubleAttr_ObjVal);
    }
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
    local_model_.update();

    for (auto constraint_number = local_model_.get(GRB_IntAttr_NumConstrs) - 1; constraint_number >= 0; --constraint_number) {
        auto constraint = local_model_.getConstr(constraint_number);
        local_model_.remove(constraint);
    }

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

    for (const auto &station : remote_model_.ObservableStations()) {
        double total_keys_transferred = GetSolutionMaxKeyRate(station);

        std::stringstream constraint_label;
        constraint_label << "master_constraint_" << station.name();
        local_model_.addConstr(constraint_expr >= -total_keys_transferred / remote_model_.TransferShare(station), constraint_label.str());
    }

    local_model_.update();
    local_model_.reset();
    CHECK_EQ(local_model_.get(GRB_IntAttr_NumConstrs), remote_model_.ObservableStations().size());
}

void quake::SocMeanStdMipModel::MasterCallback::SolveUncertainty() {
    GRBModel uncertain_model_{remote_model_.mip_environment_};
    uncertain_model_.set(GRB_IntParam_OutputFlag, 0);

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

    auto cuts_added = 0;
    for (const auto &master_station : remote_model_.ObservableStations()) {
        const auto master_station_index = remote_model_.Index(master_station);

        // compute traffic index for the master station
        {
            GRBLinExpr total_keys_transferred = 0.0;
            for (const auto &period : remote_model_.ObservableCloudCover(master_station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                total_keys_transferred
                        += GetSolutionMaxKeyRate(master_station, period) * (1.0 - un_cloud_cover.at(master_station_index).at(period_index));
            }

            // configure objective
            GRBLinExpr objective = probability_dual_.get(GRB_DoubleAttr_X);
            for (const auto &local_station : remote_model_.Stations()) {
                if (local_station == GroundStation::None) { continue; }

                const auto local_station_index = remote_model_.Index(local_station);
                for (const auto &period : remote_model_.CloudCover(local_station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    objective += mean_dual_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X) *
                                 un_cloud_cover.at(local_station_index).at(period_index);
                    objective += variance_dual_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X) *
                                 un_variance.at(local_station_index).at(period_index);
                }
            }
            objective += total_keys_transferred / remote_model_.TransferShare(master_station);

            uncertain_model_.setObjective(objective);
            uncertain_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        }
        uncertain_model_.optimize();

        const auto solver_status = static_cast<util::SolverStatus >(uncertain_model_.get(GRB_IntAttr_Status));
        CHECK(solver_status == util::SolverStatus::Optimal || solver_status == util::SolverStatus::Suboptimal);
        if (util::is_surely_lt(uncertain_model_.get(GRB_DoubleAttr_ObjVal), 0.0)) {
            // compute traffic index for the master station
            double total_keys_transferred_value = 0.0;
            for (const auto &period : remote_model_.ObservableCloudCover(master_station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                total_keys_transferred_value
                        += GetSolutionMaxKeyRate(master_station, period) *
                           (1.0 - un_cloud_cover.at(master_station_index).at(period_index).get(GRB_DoubleAttr_X));
            }

            // configure objective
            double objective_value = probability_dual_.get(GRB_DoubleAttr_X);
            GRBLinExpr objective_expr = probability_dual_;
            for (const auto &local_station : remote_model_.Stations()) {
                if (local_station == GroundStation::None) { continue; }

                const auto local_station_index = remote_model_.Index(local_station);
                for (const auto &period : remote_model_.CloudCover(local_station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    objective_expr += mean_dual_.at(local_station_index).at(period_index) *
                                      un_cloud_cover.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);
                    objective_expr += variance_dual_.at(local_station_index).at(period_index) *
                                      un_variance.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);

                    objective_value += mean_dual_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X) *
                                       un_cloud_cover.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);
                    objective_value += variance_dual_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X) *
                                       un_variance.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);
                }
            }
            objective_expr += total_keys_transferred_value / remote_model_.TransferShare(master_station);
            objective_value += total_keys_transferred_value / remote_model_.TransferShare(master_station);

            LOG_IF(FATAL, !util::is_nearly_eq(objective_value, uncertain_model_.get(GRB_DoubleAttr_ObjVal)))
                            << "Objective value recomputed in uncertain model and the sub-problem model should match: "
                            << objective_value << " v.s. " << uncertain_model_.get(GRB_DoubleAttr_ObjVal)
                            << " Gap: " << abs(objective_value - uncertain_model_.get(GRB_DoubleAttr_ObjVal));

            LOG(INFO) << "Adding cut in sub-problem for station " << master_station << ": " << objective_value << " >= " << 0;
            local_model_.addConstr(objective_expr >= 0);
            ++cuts_added;
        }
    }

    if (cuts_added == 0) {
        auto traffic_index = std::numeric_limits<double>::max();
        for (const auto &station : remote_model_.ObservableStations()) {
            const auto station_index = remote_model_.Index(station);

            double total_keys_transferred_value = 0.0;
            for (const auto &period : remote_model_.ObservableCloudCover(station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                total_keys_transferred_value += GetSolutionMaxKeyRate(station, period)
                                                * (1.0 - un_cloud_cover.at(station_index).at(period_index).get(GRB_DoubleAttr_X));
            }

            const auto station_traffic_index = total_keys_transferred_value / remote_model_.TransferShare(station);
            traffic_index = std::min(station_traffic_index, traffic_index);
        }

        VLOG_IF(1, traffic_index > 0) << "Traffic Index: " << traffic_index;
    }
}
