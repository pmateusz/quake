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
#include "util/math.h"

#include "robust_average_mip_model.h"

quake::RobustAverageMipModel::RobustAverageMipModel(const ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseIntervalMipModel(problem,
                               std::move(interval_step),
                               std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {}

void quake::RobustAverageMipModel::Build(const boost::optional<Solution> &solution) {
    // set solver configuration:
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseIntervalMipModel::Build(solution);

    // variables: cloud cover
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

        dual_station_.resize(Stations().size());
        dual_cloud_cover_.resize(Stations().size());
        for (const auto &station : ObservableStations()) {
            const auto station_index = Index(station);
            auto &station_dual_cloud_cover = dual_cloud_cover_.at(station_index);
            for (const auto &period : cloud_cover_index_) {
                std::stringstream dual_cloud_cover_label;
                dual_cloud_cover_label << "dual_s" << station_index << "t" << period.begin();
                station_dual_cloud_cover.emplace_back(mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY,
                                                                        0.0, GRB_CONTINUOUS, dual_cloud_cover_label.str()));
            }

            std::stringstream dual_station_label;
            dual_station_label << "dual_s" << station_index;
            dual_station_.at(station_index) = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY,
                                                                0.0, GRB_CONTINUOUS, dual_station_label.str());
        }
    }

    // get lower bound on the objective to avoid unbounded problem in Gurobi
    auto objective_lb = std::numeric_limits<double>::max();
    for (const auto &station : ObservableStations()) {
        auto max_keys_transferred = 0.0;
        for (const auto &station_interval :StationIntervals(station)) {
            max_keys_transferred += problem_->KeyRate(station, station_interval.Period());
        }
        auto station_objective_lb = -max_keys_transferred / problem_->TransferShare(station);
        objective_lb = std::min(objective_lb, station_objective_lb);
    }

    GRBVar global_min_var = mip_model_.addVar(objective_lb, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);
        const auto num_variables = cloud_cover_index_.size();

        GRBLinExpr left_objective_reformulation = dual_station_.at(station_index);
        for (auto variable_index = 0; variable_index < num_variables; ++variable_index) {
            const auto &period = cloud_cover_index_.at(variable_index);
            left_objective_reformulation +=
                    dual_cloud_cover_.at(station_index).at(variable_index) * forecast.GetCloudCover(station, period.begin()) / 100.0;
        }

        mip_model_.addConstr(left_objective_reformulation <= global_min_var);
    }

    GRBLinExpr objective = global_min_var;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mip_model_.setObjective(objective);

    callback_ = std::make_unique<RobustAverageMipCallback>(*this);
    mip_model_.setCallback(callback_.get());
}

void quake::RobustAverageMipModel::ReportResults(quake::util::SolverStatus solver_status) {
    BaseMipModel::ReportResults(solver_status);

    const auto &forecast = problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);

    const auto num_variables = cloud_cover_index_.size();
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);
        auto left_site = dual_station_.at(station_index).get(GRB_DoubleAttr_X);

        for (auto variable_index = 0; variable_index < num_variables; ++variable_index) {
            const auto &period = cloud_cover_index_.at(variable_index);
            left_site += dual_cloud_cover_.at(station_index).at(variable_index).get(GRB_DoubleAttr_X)
                         * forecast.GetCloudCover(station, period.begin()) / 100.0;
        }

        double keys_transferred = 0;
        for (const auto &interval : StationIntervals(station)) {
            keys_transferred += problem_->KeyRate(station, interval.Period(), forecast);
        }
        double right_site = -keys_transferred / problem_->TransferShare(station);

        util::is_surely_gt(left_site, right_site);
        LOG(INFO) << "Constraint: " << left_site << " >= " << right_site;
    }

    LOG(INFO) << "Objective: " << mip_model_.get(GRB_DoubleAttr_ObjVal);
}

std::size_t quake::RobustAverageMipModel::GetCloudCoverIndex(const boost::posix_time::time_period &period) const {
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


quake::RobustAverageMipModel::RobustAverageMipCallback::RobustAverageMipCallback(quake::RobustAverageMipModel &model)
        : model_{model} {}

void quake::RobustAverageMipModel::RobustAverageMipCallback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);
        const auto num_variables = model_.cloud_cover_index_.size();

        const auto cloud_cover_assignment = SolveKeysTransferredConstraint(station);
        if (!cloud_cover_assignment.empty()) {
            GRBLinExpr left_constraint_expr = model_.dual_station_.at(station_index);

            double left_constraint_value = getSolution(model_.dual_station_.at(station_index));
            for (auto variable_index = 0; variable_index < num_variables; ++variable_index) {
                left_constraint_expr
                        += model_.dual_cloud_cover_.at(station_index).at(variable_index) * cloud_cover_assignment.at(variable_index);
                left_constraint_value
                        += getSolution(model_.dual_cloud_cover_.at(station_index).at(variable_index)) * cloud_cover_assignment.at(variable_index);
            }

            GRBLinExpr keys_transferred_expr = 0;
            double keys_transferred_value = 0;
            for (const auto &interval : model_.StationIntervals(station)) {
                const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());

                const auto interval_keys_transferred = (1.0 - cloud_cover_assignment.at(cloud_cover_index))
                                                       * model_.problem_->KeyRate(station, interval.Period());
                keys_transferred_expr += interval.Var() * interval_keys_transferred;
                keys_transferred_value += getSolution(interval.Var()) * interval_keys_transferred;
            }
            GRBLinExpr right_constraint_expr = -keys_transferred_expr / model_.TransferShare(station);
            double right_constraint_value = -keys_transferred_value / model_.TransferShare(station);

            VLOG(1) << "KeyRateCut - Station " << station << ": " << left_constraint_value << " >= " << right_constraint_value;
            addLazy(left_constraint_expr >= right_constraint_expr);
        }
    }
}

std::vector<GRBVar> quake::RobustAverageMipModel::RobustAverageMipCallback::CreateCloudCoverVariables(
        const GroundStation &station, GRBModel &model) const {
    const auto num_variables = model_.cloud_cover_index_.size();

    if (num_variables == 0) {
        return {};
    }

    static const auto max_relative_deviation = 0.3;
    const auto step_duration = model_.cloud_cover_index_.front().length().total_seconds();
    const auto total_duration = (model_.cloud_cover_index_.back().end() - model_.cloud_cover_index_.front().begin()).total_seconds();

    const auto &forecast = model_.problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);
    std::vector<GRBVar> cloud_cover_vars;
    cloud_cover_vars.reserve(num_variables);
    for (auto variable_index = 0; variable_index < num_variables; ++variable_index) {
        const auto bound_deviation = step_duration * variable_index * max_relative_deviation / total_duration;
        const auto cloud_cover = forecast.GetCloudCover(station, model_.cloud_cover_index_.at(variable_index).begin()) / 100.0;
        const auto min_value = std::max(0.0, cloud_cover - bound_deviation);
        const auto max_value = std::min(1.0, cloud_cover + bound_deviation);

        cloud_cover_vars.emplace_back(model.addVar(min_value, max_value, 0.0, GRB_CONTINUOUS));
    }

    return cloud_cover_vars;
}

std::vector<double> quake::RobustAverageMipModel::RobustAverageMipCallback::SolveKeysTransferredConstraint(const quake::GroundStation &station) {
    const auto station_index = model_.Index(station);
    const auto num_variables = model_.cloud_cover_index_.size();

    const auto &forecast = model_.problem_->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast);
    GRBModel local_model{model_.mip_environment_};

    std::vector<GRBVar> cloud_cover_vars = CreateCloudCoverVariables(station, local_model);

    GRBLinExpr keys_transferred_expr = 0;
    for (const auto &interval : model_.StationIntervals(station)) {
        const auto cloud_cover_index = model_.GetCloudCoverIndex(interval.Period());
        keys_transferred_expr += getSolution(interval.Var())
                                 * model_.problem_->KeyRate(station, interval.Period())
                                 * (1.0 - cloud_cover_vars.at(cloud_cover_index));
    }

    GRBLinExpr left_constraint_expr = getSolution(model_.dual_station_.at(station_index));
    for (auto variable_index = 0; variable_index < num_variables; ++variable_index) {
        left_constraint_expr += getSolution(model_.dual_cloud_cover_.at(station_index).at(variable_index)) * cloud_cover_vars.at(variable_index);
    }

    GRBLinExpr objective = left_constraint_expr + keys_transferred_expr / model_.TransferShare(station);
    local_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    local_model.setObjective(objective);
    local_model.set(GRB_IntParam_OutputFlag, 0);
    local_model.optimize();

    const auto solver_status = static_cast<util::SolverStatus>(local_model.get(GRB_IntAttr_Status));
    CHECK_EQ(solver_status, util::SolverStatus::Optimal);

    if (util::is_surely_lt(local_model.get(GRB_DoubleAttr_ObjVal), 0)) {
        std::vector<double> assignment;
        assignment.reserve(cloud_cover_vars.size());

        for (const auto &cloud_cover_var : cloud_cover_vars) {
            assignment.emplace_back(cloud_cover_var.get(GRB_DoubleAttr_X));
        }

        return assignment;
    }

    return {};
}
