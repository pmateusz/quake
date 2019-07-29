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

void quake::RobustVarIndexMipModel::Build(const boost::optional<Solution> &solution) {
    // set solver configuration:
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseIntervalMipModel::Build(solution);

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

    const auto num_time_steps = cloud_cover_index_.size();
    const auto &forecast = problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);
    GRBLinExpr left_objective_reformulation = dual_intercept_;
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);
        for (auto time_index = 0; time_index < num_time_steps; ++time_index) {
            const auto &period = cloud_cover_index_.at(time_index);
            left_objective_reformulation +=
                    dual_cloud_cover_by_station_.at(station_index).at(time_index) * forecast.GetCloudCover(station, period.begin()) / 100.0;
        }
    }

    // debug ->
    LOG(WARNING) << "debug only";
    mip_model_.addConstr(left_objective_reformulation >= -10);
    // <-

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
          traffic_index_upper_bound_{model.GetTrafficIndexUpperBound()} {}

void quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    GRBModel local_model{model_.mip_environment_};
    const auto num_time_steps = model_.cloud_cover_index_.size();

    auto cloud_cover_vars = CreateCloudCoverVariables(local_model);
    GRBVar traffic_index = local_model.addVar(0.0, traffic_index_upper_bound_, 0.0, GRB_CONTINUOUS);
    {
        // solve the combined sub-problem for all observable stations

        for (const auto &station : model_.ObservableStations()) {
            const auto station_index = model_.Index(station);

            GRBLinExpr keys_transferred_expr = 0;
            for (const auto &interval : model_.StationIntervals(station)) {
                const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
                keys_transferred_expr += getSolution(interval.Var())
                                         * model_.problem_->KeyRate(station, interval.Period())
                                         * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index));
            }

            local_model.addConstr(model_.TransferShare(station) * traffic_index <= keys_transferred_expr);
        }

        // right-hand side of the dual constraint
        GRBLinExpr right_constraint_expr = model_.target_index_ - traffic_index;

        // left-hand side of the dual constraint
        GRBLinExpr left_constraint_expr = getSolution(model_.dual_intercept_);
        for (const auto &station : model_.ObservableStations()) {
            const auto station_index = model_.Index(station);

            for (auto time_step = 0; time_step < num_time_steps; ++time_step) {
                left_constraint_expr
                        += getSolution(model_.dual_cloud_cover_by_station_.at(station_index).at(time_step))
                           * cloud_cover_vars.at(station_index).at(time_step);
            }
        }

        GRBLinExpr objective = left_constraint_expr - right_constraint_expr;
        local_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        local_model.setObjective(objective);
        local_model.set(GRB_IntParam_OutputFlag, 0);
        local_model.optimize();
    }
    const auto solver_status = static_cast<util::SolverStatus>(local_model.get(GRB_IntAttr_Status));
    CHECK_EQ(solver_status, util::SolverStatus::Optimal);

    LOG(INFO) << "Cut test: " << local_model.get(GRB_DoubleAttr_ObjVal) << " v.s " << 0.0;
    if (util::is_surely_gt(local_model.get(GRB_DoubleAttr_ObjVal), 0.0)) {
        LOG(INFO) << "No optimality cuts needed";
        return;
    }

// //     debug code
//    {
//        GRBLinExpr left_constraint_expr = model_.dual_intercept_;
//        for (const auto &station : model_.ObservableStations()) {
//            const auto station_index = model_.Index(station);
//
//            for (auto time_step = 0; time_step < num_time_steps; ++time_step) {
//                left_constraint_expr += model_.dual_cloud_cover_by_station_.at(station_index).at(time_step)
//                                        * cloud_cover_vars.at(station_index).at(time_step).get(GRB_DoubleAttr_X);
//            }
//        }
//
//        addLazy(left_constraint_expr >= -10);
//        return;
//    }


    // cuts needs to be generated for each station where traffic index is tight

    // left-hand side of the dual constraint
    GRBLinExpr left_constraint_expr = model_.dual_intercept_;
    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);

        for (auto time_step = 0; time_step < num_time_steps; ++time_step) {
            left_constraint_expr += model_.dual_cloud_cover_by_station_.at(station_index).at(time_step)
                                    * cloud_cover_vars.at(station_index).at(time_step).get(GRB_DoubleAttr_X);
        }
    }

    // TODO: use only those could cover variables which appear in intervals, otherwise problem will be unbounded!

    // find stations where traffic index is tight
    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);

        double keys_transferred = 0.0;
        for (const auto &interval : model_.StationIntervals(station)) {
            const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
            keys_transferred += getSolution(interval.Var())
                                * model_.problem_->KeyRate(station, interval.Period())
                                * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
        }

        LOG(INFO) << station << " transferred " << keys_transferred;
        LOG(INFO) << model_.TransferShare(station) * traffic_index.get(GRB_DoubleAttr_X) << " " << keys_transferred;
        if (util::is_nearly_eq(model_.TransferShare(station) * traffic_index.get(GRB_DoubleAttr_X), keys_transferred)) {
            // traffic index is tight

            // generate lazy constraint
            GRBLinExpr keys_transferred_expr = 0.0;
            for (const auto &interval : model_.StationIntervals(station)) {
                const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
                keys_transferred_expr += interval.Var()
                                         * model_.problem_->KeyRate(station, interval.Period())
                                         * (1.0 - cloud_cover_vars.at(station_index).at(cloud_cover_index).get(GRB_DoubleAttr_X));
            }

            // feasibility cuts
            addLazy(left_constraint_expr >= model_.target_index_ - keys_transferred_expr / model_.TransferShare(station));
        }
    }
}

std::vector<std::vector<GRBVar> > quake::RobustVarIndexMipModel::RobustVarIndexMipCallback::CreateCloudCoverVariables(GRBModel &model) {
    std::vector<std::vector<GRBVar>> cloud_cover_variables;
    cloud_cover_variables.resize(model_.Stations().size());

    const auto num_time_steps = model_.cloud_cover_index_.size();
    for (const auto &station : model_.ObservableStations()) {
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

    return cloud_cover_variables;
}
