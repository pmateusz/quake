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

#include "box_benders_mip_model.h"

quake::BoxBendersMipModel::BoxBendersMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {}

void quake::BoxBendersMipModel::Build(const boost::optional<Solution> &solution) {
    mip_model_.set(GRB_IntParam_LazyConstraints, 1);

    BaseIntervalMipModel::Build(solution);

    GRBVar traffic_index = mip_model_.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
    for (const auto &station: ObservableStations()) {
        GRBLinExpr keys_transferred = 0;
        for (const auto &interval : StationIntervals(station)) {
            const auto mean_keys_transferred
                    = problem_->KeyRate(station, interval.Period()) * (1.0 - CloudCoverUpperBound(station, interval.Period()));
            keys_transferred += mean_keys_transferred * interval.Var();
        }
        mip_model_.addConstr(TransferShare(station) * traffic_index <= keys_transferred);
    }

    upper_bound_ = mip_model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);
    mip_model_.addConstr(upper_bound_ >= -traffic_index);

    GRBLinExpr objective = upper_bound_;
    mip_model_.setObjective(objective);
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    callback_ = std::make_unique<Callback>(*this);
    mip_model_.setCallback(callback_.get());
}

quake::BoxBendersMipModel::Callback::Callback(quake::BoxBendersMipModel &model)
        : model_{model} {}

void quake::BoxBendersMipModel::Callback::callback() {
    if (where != GRB_CB_MIPSOL) { return; }

    const auto &observable_stations = model_.ObservableStations();
    if (observable_stations.empty()) { return; }

    const auto num_stations = model_.Stations().size();
    const auto num_cloud_cover_periods = model_.CloudCover(*observable_stations.begin()).size();

    GRBModel callback_model{model_.mip_environment_};
    std::vector<GRBVar> traffic_index_duals = util::CreateVarVector(callback_model, num_stations,
                                                                    0, GRB_INFINITY, "traffic_dual");
    std::vector<std::vector<GRBVar> > cloud_cover_duals = util::CreateVarMatrix(callback_model, num_stations, num_cloud_cover_periods,
                                                                                -GRB_INFINITY, GRB_INFINITY, "cc_dual");

    GRBLinExpr convex_combination = 0;
    GRBLinExpr objective = 0;
    for (const auto &station : model_.ObservableStations()) {
        const auto station_index = model_.Index(station);

        convex_combination += model_.TransferShare(station) * traffic_index_duals.at(station_index);

        for (const auto &period: model_.ObservableCloudCover(station)) {
            double keys_transferred = 0.0;
            for (const auto &interval : model_.GetIntervals(station, period)) {
                keys_transferred += getSolution(interval.Var()) * model_.problem_->KeyRate(station, interval.Period());
            }

            const auto period_index = model_.CloudCoverIndex(period);
            objective -= keys_transferred * traffic_index_duals.at(station_index);
            objective += keys_transferred * cloud_cover_duals.at(station_index).at(period_index);

            GRBLinExpr lower_bound_expr = -model_.CloudCoverLowerBound(station, period) * traffic_index_duals.at(station_index)
                                          + cloud_cover_duals.at(station_index).at(period_index);
            callback_model.addConstr(lower_bound_expr >= 0);

            GRBLinExpr upper_bound_expr = model_.CloudCoverUpperBound(station, period) * traffic_index_duals.at(station_index)
                                          - cloud_cover_duals.at(station_index).at(period_index);
            callback_model.addConstr(upper_bound_expr >= 0);
        }
    }
    callback_model.addConstr(convex_combination >= 1.0);

    callback_model.setObjective(objective);
    callback_model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    callback_model.optimize();
    const auto solver_status_code = callback_model.get(GRB_IntAttr_Status);
    const auto solver_status = static_cast<util::SolverStatus>(solver_status_code);
    VLOG(1) << "Solving sub-problem status: " << solver_status;
    CHECK_NE(solver_status, util::SolverStatus::Infeasible);

    if (solver_status == util::SolverStatus::InfiniteOrUnbounded) {
        LOG(FATAL) << "Feasibility cut needed";
    } else if (solver_status == util::SolverStatus::Optimal) {
        const auto sub_problem_value = callback_model.get(GRB_DoubleAttr_ObjVal);
        const auto master_problem_value = getSolution(model_.upper_bound_);
        if (sub_problem_value != master_problem_value) {
            if (sub_problem_value > master_problem_value) {
                VLOG(1) << "Adding cut: " << master_problem_value << " >= " << sub_problem_value;

                // the master problem should act as an upper bound
                GRBLinExpr sub_problem_expr = 0;
                for (const auto &station : model_.ObservableStations()) {
                    const auto station_index = model_.Index(station);
                    for (const auto &period: model_.ObservableCloudCover(station)) {
                        const auto period_index = model_.CloudCoverIndex(period);

                        GRBLinExpr keys_transferred = 0.0;
                        for (const auto &interval : model_.GetIntervals(station, period)) {
                            keys_transferred += interval.Var() * model_.problem_->KeyRate(station, interval.Period());
                        }

                        const auto coefficient = -traffic_index_duals.at(station_index).get(GRB_DoubleAttr_X)
                                                 + cloud_cover_duals.at(station_index).at(period_index).get(GRB_DoubleAttr_X);
                        sub_problem_expr += coefficient * keys_transferred;
                    }
                }

                addLazy(sub_problem_expr >= sub_problem_value);
            } else {
                VLOG(1) << "Approximation has a small gap. " << sub_problem_value
                        << " (sub-problem) v.s. " << master_problem_value
                        << " (master problem)";
            }
        } else {
            VLOG(1) << "No more cuts needed. Master problem and sub-problem values agree.";
        }
    }
}
