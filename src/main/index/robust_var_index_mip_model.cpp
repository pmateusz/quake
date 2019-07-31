//
// Copyright 2019 Mateusz Polnik
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <unordered_set>
#include "robust_var_index_mip_model.h"

#include "util/math.h"

quake::RobustVarIndexMipModel::RobustVarIndexMipModel(const quake::ExtendedProblem *problem,
                                                      boost::posix_time::time_duration interval_step,
                                                      double target_index)
        : BaseIntervalMipModel(problem,
                               std::move(interval_step),
                               std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}),
          target_index_{target_index} {

    const auto &forecast = problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);
    const auto &forecast_index = forecast.Index();
    if (!forecast_index.empty()) {
        const auto &series = forecast_index.begin()->second;
        std::vector<boost::posix_time::time_period> time_periods;
        for (auto period = series.Period().begin(); period < series.Period().end(); period += series.UpdateFrequency()) {
            time_periods.emplace_back(boost::posix_time::time_period{period, period + series.UpdateFrequency()});
        }
        cloud_cover_index_ = std::move(time_periods);
        CHECK_EQ(cloud_cover_index_.size(), forecast.Index().begin()->second.Values().size());
    }
}

const double MY_GRB_INFINITY = 10e30;

void quake::RobustVarIndexMipModel::Build(const boost::optional<Solution> &solution) {
    // set solver configuration:
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseIntervalMipModel::Build(solution);

    // initialize used cloud cover indices
    used_cloud_cover_indices_.resize(Stations().size());
    for (const auto &station :ObservableStations()) {
        std::unordered_set<std::size_t> indices;
        for (const auto &interval : StationIntervals(station)) {
            indices.insert(GetCloudCoverIndex(interval.Period()));
        }

        std::vector<std::size_t> sorted_indices(std::cbegin(indices), std::cend(indices));
        std::sort(std::begin(sorted_indices), std::end(sorted_indices));
        used_cloud_cover_indices_.at(Index(station)) = std::move(sorted_indices);
    }

    // variable: dual intercept
    dual_intercept_ = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "dual_intercept");

    // variables: cloud cover for each station
    dual_cloud_cover_by_station_.resize(Stations().size());
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);
        auto &dual_cloud_cover_station_row = dual_cloud_cover_by_station_.at(station_index);
        for (const auto &period : cloud_cover_index_) {
            std::stringstream dual_cloud_cover_label;
            dual_cloud_cover_label << "dual_s" << station_index << "t" << period.begin();
            dual_cloud_cover_station_row.emplace_back(
                    mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, dual_cloud_cover_label.str()));
        }
    }

    const auto &forecast = problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);
    GRBLinExpr left_objective_reformulation = dual_intercept_;
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);
        for (const auto cloud_cover_index : used_cloud_cover_indices_.at(station_index)) {
            const auto &period = cloud_cover_index_.at(cloud_cover_index);
            left_objective_reformulation +=
                    dual_cloud_cover_by_station_.at(station_index).at(cloud_cover_index) * forecast.GetCloudCover(station, period.begin()) / 100.0;
        }
    }

    const auto traffic_index_ub = GetTrafficIndexUpperBound();
    mip_model_.addConstr(left_objective_reformulation >= -traffic_index_ub); // valid lower bound for the expression above

    GRBLinExpr objective = left_objective_reformulation;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mip_model_.setObjective(objective);

    callback_ = std::make_unique<RobustVarIndexMipCallback>(*this);
    mip_model_.setCallback(callback_.get());
}

std::size_t quake::RobustVarIndexMipModel::GetCloudCoverIndex(const boost::posix_time::time_period &period) const {
    auto found = false;
    auto variable_index = 0;
    for (const auto &cloud_cover_period : cloud_cover_index_) {
        if (cloud_cover_period.contains(period)) {
            found = true;
            break;
        }

        CHECK(!cloud_cover_period.is_after(period.begin()));
        ++variable_index;
    }

    CHECK(found);
    return variable_index;
}

quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::RobustVarIndexMipCallback(quake::RobustVarIndexMipModel &model)
        : model_{model},
          traffic_index_upper_bound_{model.GetTrafficIndexUpperBound()} {
}

void quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::callback_slack() {
    if (where != GRB_CB_MIPSOL) { return; }

    GRBModel local_model{model_.mip_environment_};
    std::vector<std::vector<GRBVar>> cloud_cover_vars = CreateCloudCoverVariables(local_model);

    GRBLinExpr left_hand_side_expr = getSolution(model_.dual_intercept_);
    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);
        for (const auto cloud_cover_index: model_.used_cloud_cover_indices_.at(station_index)) {
            left_hand_side_expr += getSolution(model_.dual_cloud_cover_by_station_.at(station_index).at(cloud_cover_index))
                                   * cloud_cover_vars.at(station_index).at(cloud_cover_index);
        }
    }

    std::vector<GRBVar> slacks;
    slacks.resize(model_.Stations().size());
    for (const auto &station : model_.ObservableStations()) {
        slacks.at(model_.Index(station)) = local_model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
    }

    local_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    local_model.set(GRB_IntParam_NumericFocus, 2);
    local_model.set(GRB_IntParam_OutputFlag, 0);

    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);
        GRBLinExpr keys_transferred = 0.0;
        for (const auto &interval : model_.StationIntervals(station)) {
            const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
            keys_transferred += getSolution(interval.Var())
                                * model_.problem_->KeyRate(station, interval.Period())
                                * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index));
        }

        local_model.addConstr(left_hand_side_expr
                              <= model_.target_index_ - keys_transferred / model_.TransferShare(station) + slacks.at(station_index));
    }

    bool any_cuts = false;
    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);

        GRBLinExpr objective = slacks.at(station_index);
        local_model.setObjective(objective);
        local_model.optimize();

        double master_left_value = getSolution(model_.dual_intercept_);
        for (const auto &master_station : model_.ObservableStations()) {
            const auto master_station_index = model_.Index(master_station);
            for (const auto cloud_cover_index: model_.used_cloud_cover_indices_.at(master_station_index)) {
                master_left_value += getSolution(model_.dual_cloud_cover_by_station_.at(master_station_index).at(cloud_cover_index))
                                     * cloud_cover_vars.at(master_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X);
            }
        }

        double master_keys_transferred_value = 0.0;
        for (const auto &interval : model_.StationIntervals(station)) {
            const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
            master_keys_transferred_value += getSolution(interval.Var())
                                             * model_.problem_->KeyRate(station, interval.Period())
                                             * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
        }

        VLOG(1) << local_model.get(GRB_DoubleAttr_ObjVal);
        if (util::is_surely_lt(local_model.get(GRB_DoubleAttr_ObjVal), 0.0)) {
            GRBLinExpr master_left_expr = model_.dual_intercept_;
            for (const auto &master_station : model_.ObservableStations()) {
                const auto master_station_index = model_.Index(master_station);
                for (const auto cloud_cover_index: model_.used_cloud_cover_indices_.at(master_station_index)) {
                    master_left_expr += model_.dual_cloud_cover_by_station_.at(master_station_index).at(cloud_cover_index)
                                        * cloud_cover_vars.at(master_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X);
                }
            }

            GRBLinExpr master_keys_transferred_expr = 0.0;
            for (const auto &interval : model_.StationIntervals(station)) {
                const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
                master_keys_transferred_expr += interval.Var()
                                                * model_.problem_->KeyRate(station, interval.Period())
                                                * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
            }

            addLazy(master_left_expr >= model_.target_index_ - master_keys_transferred_expr / model_.TransferShare(station));
            any_cuts = true;

            CHECK(util::is_surely_lt(master_left_value, model_.target_index_ - master_keys_transferred_value / model_.TransferShare(station)));

            VLOG(1) << "Cut at " << station << ": "
                    << master_left_value << " >= " << model_.target_index_ - master_keys_transferred_value / model_.TransferShare(station);
        } else {
            VLOG(1) << "Cut not needed " << station << ": "
                    << master_left_value << " >= " << model_.target_index_ - master_keys_transferred_value / model_.TransferShare(station);
        }
    }

    if (!any_cuts) {
        LOG(INFO) << "No cuts added";
    }
}

// optimization of the traffic index alone - best is at zero
void quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::callback_best_at_zero() {
    if (where != GRB_CB_MIPSOL) { return; }

    GRBModel local_model{model_.mip_environment_};
    std::vector<std::vector<GRBVar>> cloud_cover_vars = CreateCloudCoverVariables(local_model);

    GRBVar traffic_index = local_model.addVar(0, this->traffic_index_upper_bound_, 0, GRB_CONTINUOUS);
    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);
        GRBLinExpr station_keys_transferred = 0.0;
        for (const auto &interval : model_.StationIntervals(station)) {
            const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
            station_keys_transferred += getSolution(interval.Var())
                                        * model_.problem_->KeyRate(station, interval.Period())
                                        * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index));
        }

        local_model.addConstr(model_.TransferShare(station) * traffic_index <= station_keys_transferred);
    }

    GRBLinExpr objective = model_.target_index_ - traffic_index;
    local_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    local_model.setObjective(objective);
    local_model.set(GRB_IntParam_NumericFocus, 2);
    local_model.set(GRB_IntParam_OutputFlag, 1);
    local_model.optimize();

    const auto solver_status = static_cast<util::SolverStatus>(local_model.get(GRB_IntAttr_Status));
    CHECK_EQ(solver_status, util::SolverStatus::Optimal);

    double left_hand_side = getSolution(model_.dual_intercept_);
    for (const auto &other_station : model_.ObservableStations()) {
        const auto other_station_index = model_.Index(other_station);
        for (const auto cloud_cover_index: model_.used_cloud_cover_indices_.at(other_station_index)) {
            left_hand_side += getSolution(model_.dual_cloud_cover_by_station_.at(other_station_index).at(cloud_cover_index))
                              * cloud_cover_vars.at(other_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X);
        }
    }

    VLOG(1) << local_model.get(GRB_DoubleAttr_ObjVal) << " v.s. " << left_hand_side;
    if (util::is_nearly_eq(left_hand_side, local_model.get(GRB_DoubleAttr_ObjVal))
        || util::is_surely_gt(left_hand_side, local_model.get(GRB_DoubleAttr_ObjVal))) {
        return;
    }

    GRBLinExpr dual_left_expr = model_.dual_intercept_;
    for (const auto &other_station : model_.ObservableStations()) {
        const auto other_station_index = model_.Index(other_station);
        for (const auto cloud_cover_index: model_.used_cloud_cover_indices_.at(other_station_index)) {
            dual_left_expr += model_.dual_cloud_cover_by_station_.at(other_station_index).at(cloud_cover_index)
                              * cloud_cover_vars.at(other_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X);
        }
    }

    for (const auto &other_station : model_.ObservableStations()) {
        const auto other_station_index = model_.Index(other_station);
        GRBLinExpr keys_transferred_expr = 0.0;
        for (const auto &interval : model_.StationIntervals(other_station)) {
            const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
            keys_transferred_expr += interval.Var()
                                     * model_.problem_->KeyRate(other_station, interval.Period())
                                     * (1.0 - cloud_cover_vars.at(other_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
        }

        addLazy(dual_left_expr >= model_.target_index_ - keys_transferred_expr / model_.TransferShare(other_station));
    }
}

void quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::callback_combined() {
    if (where != GRB_CB_MIPSOL) { return; }

    double current_objective = 0;
    {
        const auto &forecast = model_.problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);
        current_objective = getSolution(model_.dual_intercept_);
        for (const auto &station : model_.ObservableStations()) {
            const auto station_index = model_.Index(station);
            for (const auto cloud_cover_index : model_.used_cloud_cover_indices_.at(station_index)) {
                const auto &period = model_.cloud_cover_index_.at(cloud_cover_index);
                current_objective += getSolution(model_.dual_cloud_cover_by_station_.at(station_index).at(cloud_cover_index))
                                     * forecast.GetCloudCover(station, period.begin()) / 100.0;
            }
        }
    }

    bool cut_generated = false;
    {
        GRBModel local_model{model_.mip_environment_};
        std::vector<std::vector<GRBVar>> cloud_cover_vars = CreateCloudCoverVariables(local_model);

        // left-hand side of the dual constraint
        GRBLinExpr left_constraint_expr = getSolution(model_.dual_intercept_);
        for (const auto &other_station : model_.ObservableStations()) {
            const auto other_station_index = model_.Index(other_station);
            for (const auto cloud_cover_index: model_.used_cloud_cover_indices_.at(other_station_index)) {
                left_constraint_expr
                        += getSolution(model_.dual_cloud_cover_by_station_.at(other_station_index).at(cloud_cover_index))
                           * cloud_cover_vars.at(other_station_index).at(cloud_cover_index);
            }
        }

        for (const auto &master_station : model_.ObservableStations()) {
            const auto master_station_index = model_.Index(master_station);

            {
                GRBLinExpr keys_transferred_expr = 0.0;
                for (const auto &interval : model_.StationIntervals(master_station)) {
                    const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
                    keys_transferred_expr += getSolution(interval.Var())
                                             * model_.problem_->KeyRate(master_station, interval.Period())
                                             * (1.0 - cloud_cover_vars.at(master_station_index).at(cloud_cover_index));
                }

                // objective
                GRBLinExpr objective = left_constraint_expr - model_.target_index_ + keys_transferred_expr / model_.TransferShare(master_station);

                // right-hand side of the dual constraint
                local_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
                local_model.setObjective(objective);
                local_model.set(GRB_IntParam_NumericFocus, 2.0);
                local_model.set(GRB_IntParam_OutputFlag, 0);
                local_model.optimize();
            }

            const auto solver_status = static_cast<util::SolverStatus>(local_model.get(GRB_IntAttr_Status));
            CHECK_EQ(solver_status, util::SolverStatus::Optimal);

            VLOG(1) << local_model.get(GRB_DoubleAttr_ObjVal) << " v.s " << 0.0;
            if (util::is_surely_lt(local_model.get(GRB_DoubleAttr_ObjVal), 0.0)) {
                VLOG(1) << "Cuts needed. Dual constraint violated by " << local_model.get(GRB_DoubleAttr_ObjVal)
                        << " at " << master_station;

                double cut_keys_transferred_value = 0.0;
                for (const auto &interval : model_.StationIntervals(master_station)) {
                    const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
                    cut_keys_transferred_value += getSolution(interval.Var())
                                                  * model_.problem_->KeyRate(master_station, interval.Period())
                                                  * (1.0 - cloud_cover_vars.at(master_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
                }

                // left-hand side of the dual constraint
                double cut_left_constraint_value = getSolution(model_.dual_intercept_);
                GRBLinExpr cut_left_constraint_expr = model_.dual_intercept_;
                for (const auto &other_station : model_.ObservableStations()) {
                    const auto other_station_index = model_.Index(other_station);
                    for (const auto cloud_cover_index : model_.used_cloud_cover_indices_.at(other_station_index)) {
                        cut_left_constraint_value += getSolution(model_.dual_cloud_cover_by_station_.at(other_station_index).at(cloud_cover_index))
                                                     * cloud_cover_vars.at(other_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X);

                        cut_left_constraint_expr += model_.dual_cloud_cover_by_station_.at(other_station_index).at(cloud_cover_index)
                                                    * cloud_cover_vars.at(other_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X);
                    }
                }

                // generate lazy constraint
                GRBLinExpr cut_keys_transferred_expr = 0.0;
                for (const auto &interval : model_.StationIntervals(master_station)) {
                    const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
                    cut_keys_transferred_expr += interval.Var()
                                                 * model_.problem_->KeyRate(master_station, interval.Period())
                                                 * (1.0 - cloud_cover_vars.at(master_station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
                }

                // feasibility cuts
                addLazy(cut_left_constraint_expr >= model_.target_index_ - cut_keys_transferred_expr / model_.TransferShare(master_station));
                VLOG(1) << "Cut: " << cut_left_constraint_value
                        << " >= " << model_.target_index_ - cut_keys_transferred_value / model_.TransferShare(master_station);
                cut_generated = true;
            }
        }
    }

    if (!cut_generated && current_objective <= -traffic_index_upper_bound_) {
        LOG(FATAL) << "Failed to generate a cut";
    }
}

std::vector<std::vector<GRBVar> > quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::CreateCloudCoverVariables(GRBModel &model) {
    std::vector<std::vector<GRBVar>> cloud_cover_variables;
    cloud_cover_variables.resize(model_.Stations().size());

    const auto num_time_steps = model_.cloud_cover_index_.size();
    for (const auto &station : model_.Stations()) {
        if (station == GroundStation::None) {
            continue;
        }

        const auto &var_model = model_.problem_->VarModel(station);
        const auto station_index = model_.Index(station);
        cloud_cover_variables.at(station_index).reserve(num_time_steps);
        for (auto time_step = 0; time_step < num_time_steps; ++time_step) {
            const auto min_value = var_model.LowerBound.at(time_step) / 100.0;
            const auto max_value = var_model.UpperBound.at(time_step) / 100.0;

            CHECK_LE(min_value, 1.0);
            CHECK_GE(min_value, 0.0);
            CHECK_LE(max_value, 1.0);
            CHECK_GE(max_value, 0.0);

            cloud_cover_variables.at(station_index).push_back(model.addVar(min_value, max_value, 0.0, GRB_CONTINUOUS));
        }
    }

    // TODO: create VAR model
    for (const auto &master_station : model_.Stations()) {
        if (master_station == GroundStation::None) {
            continue;
        }

        const auto &master_var_model = model_.problem_->VarModel(master_station);
        const auto master_station_index = model_.Index(master_station);

        for (auto time_step = 1; time_step < num_time_steps; ++time_step) {
            GRBLinExpr next_prediction = master_var_model.Intercept.Value / 100.0;
            for (const auto &other_station : model_.Stations()) {
                if (other_station == GroundStation::None) {
                    continue;
                }

                const auto other_station_index = model_.Index(other_station);
                next_prediction +=
                        master_var_model.Parameters.at(other_station).Value * cloud_cover_variables.at(other_station_index).at(time_step - 1);
            }

//            model.addConstr(cloud_cover_variables.at(master_station_index).at(time_step) <= 1.0);
//            model.addConstr(cloud_cover_variables.at(master_station_index).at(time_step) >= 0.0);
//            model.addConstr(
//                    cloud_cover_variables.at(master_station_index).at(time_step) >= next_prediction - 2 * master_var_model.Residual.Stderr / 100.0);
//            model.addConstr(
//                    cloud_cover_variables.at(master_station_index).at(time_step) <= next_prediction + 2 * master_var_model.Residual.Stderr / 100.0);
        }
    }

    return cloud_cover_variables;
}

void quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::callback() {
    callback_slack();
}
