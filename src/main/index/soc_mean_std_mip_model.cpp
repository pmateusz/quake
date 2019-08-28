#include <chrono>
#include <random>
#include "soc_mean_std_mip_model.h"
#include "util/math.h"

quake::SocMeanStdMipModel::SocMeanStdMipModel(const quake::ExtendedProblem *problem,
                                              boost::posix_time::time_duration interval_step,
                                              double confidence_radius)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}),
          confidence_radius_{confidence_radius} {}


static const auto WEIGHT_FACTOR = 1.0;

void AddVarModelConstraints(const quake::BaseRobustMipModel &model, GRBModel &mip_model, std::vector<std::vector<GRBVar> > &cloud_cover) {
    static const auto error_contrib = 2.0;
    static const auto NORMALIZATION_CONST = 100.0;

    // define VAR constraints for each station
    for (const auto &output_station : model.ObservableStations()) {
        const auto output_station_index = model.Index(output_station);
        const auto &var_model = model.VarModel(output_station);
        const auto &cloud_cover_periods = model.CloudCover(output_station);
        for (std::size_t output_index = 1; output_index < cloud_cover.at(output_station_index).size(); ++output_index) {
            auto prev_index = output_index - 1;

            // left hand-side of var model
            GRBLinExpr lower_series_expr = var_model.Intercept.Value;
            GRBLinExpr upper_series_expr = var_model.Intercept.Value;
            for (const auto &other_station: model.Stations()) {
                if (other_station == quake::GroundStation::None) {
                    continue;
                }

                const auto other_station_index = model.Index(other_station);
                const auto other_station_param = var_model.Parameters.at(other_station);
                lower_series_expr += NORMALIZATION_CONST * other_station_param.Value
                                     * cloud_cover.at(other_station_index).at(prev_index);
                upper_series_expr += NORMALIZATION_CONST * other_station_param.Value
                                     * cloud_cover.at(other_station_index).at(prev_index);
            }

            mip_model.addConstr(lower_series_expr - error_contrib * (var_model.Residual.Value + var_model.Residual.Stderr)
                                <= NORMALIZATION_CONST * cloud_cover.at(output_station_index).at(output_index));
            mip_model.addConstr(NORMALIZATION_CONST * cloud_cover.at(output_station_index).at(output_index) <=
                                upper_series_expr + error_contrib * (var_model.Residual.Value + var_model.Residual.Stderr));
        }
    }
}

void quake::SocMeanStdMipModel::Build(const boost::optional<Solution> &solution) {
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);
    mip_model_.set(GRB_IntParam_ScaleFlag, 2);
//    mip_model_.set(GRB_IntParam_NumericFocus, 3);

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

            mip_model_.addConstr(WEIGHT_FACTOR * TransferShare(station) * relaxed_constraint_expr >= -GetMaxKeysTransferredExpr(station));
        }

        // configure traffic index mean
        // constraint on average realization is making things worse - does it affect the solution?
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            GRBLinExpr mean_keys_transferred = 0.0;
            for (const auto &period : CloudCover(station)) {
                mean_keys_transferred += (1.0 - CloudCoverMean(station, period)) * GetMaxKeysTransferredExpr(station, period);
            }

            mip_model_.addConstr(WEIGHT_FACTOR * TransferShare(station) * relaxed_constraint_expr >= -mean_keys_transferred);
        }
    }

    {
        // configure worst case problem -> cloud cover at all locations is 1
        GRBLinExpr relaxed_constraint_expr = probability_dual_;
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            const auto station_index = Index(station);
            for (const auto &period : CloudCover(station)) {
                const auto period_index = CloudCoverIndex(period);
                relaxed_constraint_expr += mean_dual_.at(station_index).at(period_index);
                relaxed_constraint_expr += variance_dual_.at(station_index).at(period_index);
            }
        }

        // configure traffic index upper bound
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            mip_model_.addConstr(relaxed_constraint_expr >= 0);
        }

        // configure traffic index mean
        // constraint on average realization is making things worse - does it affect the solution?
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            mip_model_.addConstr(relaxed_constraint_expr >= 0);
        }
    }

    {
        // configure best case problem -> cloud cover is zero
        GRBLinExpr relaxed_constraint_expr = probability_dual_;

        // configure traffic index upper bound
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            mip_model_.addConstr(WEIGHT_FACTOR * TransferShare(station) * relaxed_constraint_expr >= -GetMaxKeysTransferredExpr(station));
        }

        // configure traffic index mean
        // constraint on average realization is making things worse - does it affect the solution?
        for (const auto &station : Stations()) {
            if (station == GroundStation::None) { continue; }

            GRBLinExpr max_keys_transferred = 0.0;
            for (const auto &period : CloudCover(station)) {
                max_keys_transferred += GetMaxKeysTransferredExpr(station, period);
            }

            mip_model_.addConstr(WEIGHT_FACTOR * TransferShare(station) * relaxed_constraint_expr >= -max_keys_transferred);
        }
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
          uncertain_model_{remote_model_.mip_environment_},
          cuts_added_{0},
          generate_all_cuts_{false} {
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

const auto SESSION_CONSTRAINT_PREFIX = "session_";

void quake::SocMeanStdMipModel::MasterCallback::SolveUncertainty() {
    // here are several implementation details:
    // we are looking for the largest violation up to certain limit for each ground station,
    // hence we are not looking for all maximum violations,
    // with lowest effort we could find first violation up to certain limit and return from the callback

    // TODO: make optional parameter in constructor
    static const auto OBJECTIVE_LB = -1.0E8;

    const auto current_cuts = cuts_added_;
    // TODO: make generator object's property
    auto master_stations = remote_model_.ObservableStations();
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(std::begin(master_stations), std::end(master_stations), std::default_random_engine(seed));

    for (const auto &master_station :master_stations) {
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
            objective += total_keys_transferred / remote_model_.TransferShare(master_station) / WEIGHT_FACTOR;

            uncertain_model_.setObjective(objective);
            uncertain_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            // TODO: make lower bound on objective a property
//            std::stringstream objective_bound_label;
//            objective_bound_label << SESSION_CONSTRAINT_PREFIX << "objective_" << master_station.name();
//            uncertain_model_.addConstr(objective >= OBJECTIVE_LB, objective_bound_label.str());
        }

        uncertain_model_.optimize();

        auto solver_status = static_cast<util::SolverStatus >(uncertain_model_.get(GRB_IntAttr_Status));
        CHECK(solver_status == util::SolverStatus::Optimal || solver_status == util::SolverStatus::Suboptimal);
        const auto cut_needed = util::is_surely_lt(uncertain_model_.get(GRB_DoubleAttr_ObjVal), 0.0);
        if (cut_needed) {
            GenerateCut(master_station);

            if (!generate_all_cuts_) {
                break;
            }
        }
    }

    if (VLOG_IS_ON(1)) {
        if (current_cuts == cuts_added_) {
            auto traffic_index = std::numeric_limits<double>::max();
            for (const auto &station : remote_model_.ObservableStations()) {
                const auto station_index = remote_model_.Index(station);

                double total_keys_transferred_value = 0.0;
                for (const auto &period : remote_model_.ObservableCloudCover(station)) {
                    const auto period_index = remote_model_.CloudCoverIndex(period);
                    total_keys_transferred_value += GetSolutionMaxKeyRate(station, period)
                                                    * (1.0 - un_cloud_cover_.at(station_index).at(period_index).get(GRB_DoubleAttr_X));
                }

                const auto station_traffic_index = total_keys_transferred_value / remote_model_.TransferShare(station) / WEIGHT_FACTOR;
                traffic_index = std::min(station_traffic_index, traffic_index);
            }

            VLOG_IF(1, traffic_index > 0) << "Traffic Index: " << traffic_index;
        }
    }
}

void quake::SocMeanStdMipModel::MasterCallback::SetupModel() {
    uncertain_model_.set(GRB_IntParam_OutputFlag, 0);
//    uncertain_model_.set(GRB_IntParam_Presolve, 1);
    uncertain_model_.set(GRB_IntParam_ScaleFlag, 2);
//    uncertain_model_.set(GRB_IntParam_NumericFocus, 3);
    uncertain_model_.set(GRB_IntParam_BarHomogeneous, 1);
//    uncertain_model_.set(GRB_IntParam_CrossoverBasis, 1);

    probability_dual_ = uncertain_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "probability_dual_m");
    mean_dual_ = util::CreateVarMatrix(uncertain_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(), -GRB_INFINITY,
                                       GRB_INFINITY, "mean_dual_m");
    variance_dual_ = util::CreateVarMatrix(uncertain_model_, remote_model_.NumStations(), remote_model_.NumCloudCoverPeriods(), 0, GRB_INFINITY,
                                           "variance_dual_m");

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

    // define VAR constraints for each station
    AddVarModelConstraints(remote_model_, uncertain_model_, un_cloud_cover_);
}

void quake::SocMeanStdMipModel::MasterCallback::ResetModel() {
    uncertain_model_.update();

    const auto num_constraints = uncertain_model_.get(GRB_IntAttr_NumConstrs);
    if (num_constraints > 0) {
        std::vector<GRBConstr> constraints_to_remove;

        for (auto constraint_index = 0; constraint_index < num_constraints; ++constraint_index) {
            const auto constraint = uncertain_model_.getConstr(constraint_index);
            if (constraint.get(GRB_StringAttr_ConstrName).rfind(SESSION_CONSTRAINT_PREFIX, 0) != std::string::npos) {
                constraints_to_remove.emplace_back(constraint);
            }
        }

        for (const auto &constraint : constraints_to_remove) {
            uncertain_model_.remove(constraint);
        }
    }
}

void quake::SocMeanStdMipModel::MasterCallback::GenerateCut(const GroundStation &master_station) {
    // compute traffic index for the master station
    const auto master_station_index = remote_model_.Index(master_station);
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
    objective_expr += total_keys_transferred_expr / remote_model_.TransferShare(master_station) / WEIGHT_FACTOR;
    objective_value += total_keys_transferred_value / remote_model_.TransferShare(master_station) / WEIGHT_FACTOR;

    CHECK_LT(objective_value, 0.0);
    VLOG(1) << "Adding cut " << master_station << ": " << objective_value << " >= 0.0";
    addLazy(objective_expr >= 0.0);
    ++cuts_added_;
}
