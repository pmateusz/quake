#include "soc_mean_std_mip_model.h"

quake::SocMeanStdMipModel::SocMeanStdMipModel(const quake::ExtendedProblem *problem, boost::posix_time::time_duration interval_step)
        : BaseRobustMipModel(problem,
                             std::move(interval_step),
                             std::vector<Forecast>{problem->GetWeatherSample(ExtendedProblem::WeatherSample::Forecast)}) {

}

void quake::SocMeanStdMipModel::Build(const boost::optional<Solution> &solution) {
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
}


