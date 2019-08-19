#include "soc_mean_std_mip_model.h"
#include "util/math.h"

quake::SocMeanStdMipModel::SocMeanStdMipModel(const quake::ExtendedProblem *problem,
                                              boost::posix_time::time_duration interval_step,
                                              double confidence_radius)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}),
          confidence_radius_{confidence_radius} {}

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
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            mip_model_.addConstr(relaxed_constraint_expr + GetMaxKeysTransferredExpr(station) / TransferShare(station) >= 0);
        }

        // configure traffic index mean
        // constraint on average realization is making things worse - does it affect the solution?
//        for (const auto &station : Stations()) {
//            if (station == GroundStation::None) { continue; }
//
//            GRBLinExpr mean_keys_transferred = 0.0;
//            for (const auto &period : CloudCover(station)) {
//                mean_keys_transferred += (1.0 - CloudCoverMean(station, period)) * GetMaxKeysTransferredExpr(station, period);
//            }
//
//            mip_model_.addConstr(relaxed_constraint_expr + mean_keys_transferred / TransferShare(station) >= 0);
//        }
    }

    callback_ = std::make_unique<MasterCallback>(*this);
    mip_model_.setCallback(callback_.get());
}

double quake::SocMeanStdMipModel::CloudCoverLowerBound(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    if (confidence_radius_ < 0) {
        return 0.0;
    }

    const auto child_lb = CloudCoverMean(station, period) - confidence_radius_ * std::sqrt(CloudCoverVariance(station, period));
    const auto lower_bound = std::max(child_lb, 0.0);

    CHECK_GE(lower_bound, 0.0);
    CHECK_LE(lower_bound, 1.0);

    return lower_bound;
}

double quake::SocMeanStdMipModel::CloudCoverUpperBound(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    if (confidence_radius_ < 0.0) {
        return 1.0;
    }

    const auto child_ub = CloudCoverMean(station, period) + confidence_radius_ * std::sqrt(CloudCoverVariance(station, period));
    const auto upper_bound = std::min(child_ub, 1.0); // take worst of these two

    CHECK_GE(upper_bound, 0.0);
    CHECK_LE(upper_bound, 1.0);

    return upper_bound;
}


quake::SocMeanStdMipModel::MasterCallback::MasterCallback(SocMeanStdMipModel &model)
        : remote_model_{model},
          uncertain_model_{remote_model_.mip_environment_} {
    SetupModel();
}

void quake::SocMeanStdMipModel::MasterCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    SolveUncertainty();
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

void quake::SocMeanStdMipModel::MasterCallback::SolveUncertainty() {
    // here are several implementation details:
    // we are looking for the largest violation up to certain limit for each ground station,
    // hence we are not looking for all maximum violations,
    // with lowest effort we could find first violation up to certain limit and return from the callback
    static const auto OBJECTIVE_LB = -10.0E12;

    auto cuts_added = 0;
    for (const auto &master_station : remote_model_.ObservableStations()) {
        const auto master_station_index = remote_model_.Index(master_station);

        ResetModel();

        // compute traffic index for the master station
        {
            GRBLinExpr total_keys_transferred = 0.0;
            for (const auto &period : remote_model_.ObservableCloudCover(master_station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                total_keys_transferred
                        += GetSolutionMaxKeyRate(master_station, period) * (1.0 - un_cloud_cover_.at(master_station_index).at(period_index));
            }

            // configure objective
            GRBLinExpr objective = getSolution(remote_model_.probability_dual_);
            for (const auto &local_station : remote_model_.Stations()) {
                if (local_station == GroundStation::None) { continue; }

                const auto local_station_index = remote_model_.Index(local_station);
                for (const auto &period : remote_model_.CloudCover(local_station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    objective += getSolution(remote_model_.mean_dual_.at(local_station_index).at(period_index))
                                 * un_cloud_cover_.at(local_station_index).at(period_index);
                    objective += getSolution(remote_model_.variance_dual_.at(local_station_index).at(period_index))
                                 * un_variance_.at(local_station_index).at(period_index);
                }
            }
            objective += total_keys_transferred / remote_model_.TransferShare(master_station);

            uncertain_model_.setObjective(objective);
            uncertain_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            uncertain_model_.addConstr(objective >= OBJECTIVE_LB);
        }
        uncertain_model_.optimize();

        auto solver_status = static_cast<util::SolverStatus >(uncertain_model_.get(GRB_IntAttr_Status));
        CHECK(solver_status == util::SolverStatus::Optimal || solver_status == util::SolverStatus::Suboptimal);
        const auto cut_needed = util::is_surely_lt(uncertain_model_.get(GRB_DoubleAttr_ObjVal), 0.0);
        if (cut_needed) {
            // compute traffic index for the master station
            double total_keys_transferred_value = 0.0;
            GRBLinExpr total_keys_transferred_expr = 0.0;
            for (const auto &period : remote_model_.ObservableCloudCover(master_station)) {
                const auto period_index = remote_model_.CloudCoverIndex(period);
                total_keys_transferred_expr
                        += remote_model_.GetMaxKeysTransferredExpr(master_station, period) *
                           (1.0 - un_cloud_cover_.at(master_station_index).at(period_index).get(GRB_DoubleAttr_X));

                total_keys_transferred_value
                        += GetSolutionMaxKeyRate(master_station, period) *
                           (1.0 - un_cloud_cover_.at(master_station_index).at(period_index).get(GRB_DoubleAttr_X));
            }

            // configure objective
            double objective_value = getSolution(remote_model_.probability_dual_);
            GRBLinExpr objective_expr = remote_model_.probability_dual_;
            for (const auto &local_station : remote_model_.Stations()) {
                if (local_station == GroundStation::None) { continue; }

                const auto local_station_index = remote_model_.Index(local_station);
                for (const auto &period : remote_model_.CloudCover(local_station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    objective_expr += remote_model_.mean_dual_.at(local_station_index).at(period_index) *
                                      un_cloud_cover_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);
                    objective_expr += remote_model_.variance_dual_.at(local_station_index).at(period_index) *
                                      un_variance_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);

                    objective_value += getSolution(remote_model_.mean_dual_.at(local_station_index).at(period_index)) *
                                       un_cloud_cover_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);
                    objective_value += getSolution(remote_model_.variance_dual_.at(local_station_index).at(period_index)) *
                                       un_variance_.at(local_station_index).at(period_index).get(GRB_DoubleAttr_X);
                }
            }
            objective_expr += total_keys_transferred_expr / remote_model_.TransferShare(master_station);
            objective_value += total_keys_transferred_value / remote_model_.TransferShare(master_station);

            CHECK_LT(objective_value, 0.0);
            VLOG(1) << "Adding cut " << master_station << ": " << objective_value << " >= 0.0";
            addLazy(objective_expr >= 0.0);
            ++cuts_added;
        }
    }

    if (VLOG_IS_ON(1)) {
        if (cuts_added == 0) {
            auto traffic_index = std::numeric_limits<double>::max();
            for (const auto &station : remote_model_.ObservableStations()) {
                const auto station_index = remote_model_.Index(station);

                double total_keys_transferred_value = 0.0;
                for (const auto &period : remote_model_.ObservableCloudCover(station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    total_keys_transferred_value += GetSolutionMaxKeyRate(station, period)
                                                    * (1.0 - un_cloud_cover_.at(station_index).at(period_index).get(GRB_DoubleAttr_X));
                }

                const auto station_traffic_index = total_keys_transferred_value / remote_model_.TransferShare(station);
                traffic_index = std::min(station_traffic_index, traffic_index);
            }

            VLOG_IF(1, traffic_index > 0) << "Traffic Index: " << traffic_index;
        }
    }
}

void quake::SocMeanStdMipModel::MasterCallback::SetupModel() {
    uncertain_model_.set(GRB_IntParam_OutputFlag, 0);

    un_variance_ = util::CreateVarMatrix(uncertain_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(),
                                         remote_model_.CloudCoverVarianceLowerBound(), GRB_INFINITY, "un_variance");

    un_cloud_cover_.resize(remote_model_.NumStations());
    for (const auto &station : remote_model_.Stations()) {
        if (station == GroundStation::None) { continue; }

        const auto station_index = remote_model_.Index(station);
        un_cloud_cover_.at(station_index).reserve(remote_model_.NumCloudCoverPeriods());
        for (const auto &period : remote_model_.CloudCover(station)) {
            un_cloud_cover_.at(station_index).emplace_back(uncertain_model_.addVar(remote_model_.CloudCoverLowerBound(station, period),
                                                                                   remote_model_.CloudCoverUpperBound(station, period),
                                                                                   0,
                                                                                   GRB_CONTINUOUS));
        }
    }

    // configure relation between variance and cloud cover
    for (const auto &station : remote_model_.Stations()) {
        if (station == GroundStation::None) { continue; }

        const auto station_index = remote_model_.Index(station);
        for (const auto &period : remote_model_.CloudCover(station)) {
            const auto period_index = remote_model_.CloudCoverIndex(period);
            const auto mean_offset = un_cloud_cover_.at(station_index).at(period_index) - remote_model_.CloudCoverMean(station, period);
            uncertain_model_.addQConstr(mean_offset * mean_offset <= un_variance_.at(station_index).at(period_index));
        }
    }
}

void quake::SocMeanStdMipModel::MasterCallback::ResetModel() {
    uncertain_model_.update();

    const auto num_constraints = uncertain_model_.get(GRB_IntAttr_NumConstrs);
    CHECK_LE(num_constraints, 1);

    if (num_constraints == 1) {
        const auto constraint = uncertain_model_.getConstr(0);
        uncertain_model_.remove(constraint);
    }
}
