#include "multi_week_block_intervals_mip_model.h"

quake::MultiWeekBlockIntervalsMipModel::MultiWeekBlockIntervalsMipModel(const quake::ExtendedProblem *problem,
                                                                        quake::Forecast forecast,
                                                                        boost::posix_time::time_duration interval_step)
        : BaseIntervalMipModel(problem, std::move(interval_step), std::vector<Forecast>{std::move(forecast)}) {}

void quake::MultiWeekBlockIntervalsMipModel::Build(const boost::optional<Solution> &solution) {
    BaseIntervalMipModel::Build(solution);

    // correction to avoid large numbers if stations are not consuming keys
    auto initial_buffer_lambda = std::numeric_limits<double>::max();
    for (const auto &station: ObservableStations()) {
        auto station_initial_lambda = InitialBuffer(station) / TransferShare(station);
        initial_buffer_lambda = std::min(initial_buffer_lambda, station_initial_lambda);
    }

    std::unordered_map<GroundStation, double> initial_buffer_to_use;
    for (const auto &station: ObservableStations()) {
        initial_buffer_to_use[station] = InitialBuffer(station) - TransferShare(station) * initial_buffer_lambda;
    }

    const auto &forecast = Forecasts().front();

    // TODO: compute max lambda for entire year
    const auto max_observation_period = problem_->ObservationPeriod();
    std::vector<double> max_station_lambda{0.0};
    for (const auto &station :Stations()) {
        if (station == GroundStation::None) { continue; }

        const double max_keys = problem_->KeyRate(station, max_observation_period, forecast);
        const auto best_station_lambda = ceil(max_keys / problem_->TransferShare(station));
        max_station_lambda.push_back(best_station_lambda);
    }

    // constraint: lambda is bounded from above
    const double max_lambda = *std::max_element(std::cbegin(max_station_lambda), std::cend(max_station_lambda));
    LOG(INFO) << max_lambda;

    // split week into periods ending every Sunday
    boost::posix_time::time_period observation_period = problem_->ObservationPeriod();
    std::vector<boost::posix_time::time_period> weeks;
    boost::posix_time::ptime current_week_start = observation_period.begin();
    while (current_week_start < observation_period.end()) {
        const auto next_week_days = 7 - current_week_start.date().day_of_week();
        boost::posix_time::ptime next_week_start{current_week_start.date() + boost::gregorian::days(next_week_days),
                                                 current_week_start.time_of_day()};

        const auto period = boost::posix_time::time_period{current_week_start, next_week_start};
        LOG(INFO) << period;
        weeks.emplace_back(period);

        current_week_start = next_week_start;
    }

    LOG(INFO) << "here";
    // TODO: have an auxiliary variable -> number of keys transferred every week for each ground station
    // TODO: define lambda for each week
    // TODO: define the objective as the maximization of the minimum lambda

//    // obtain maximum lambdas

//    lambda_ = mip_model_.addVar(0.0, max_lambda, 0.0, GRB_CONTINUOUS, "lambda");
//
//    for (const auto &station : ObservableStations()) {
//        GRBLinExpr keys_delivered = 0;
//        const auto station_index = Index(station);
//        for (const auto &interval : StationIntervals(station)) {
//            CHECK_EQ(interval.StationIndex(), station_index);
//            keys_delivered += problem_->KeyRate(station, interval.Period(), forecast) * interval.Var();
//        }
//
//        mip_model_.addConstr(TransferShare(station) * lambda_ <= initial_buffer_to_use.at(station) + keys_delivered);
//    }
//
//    // objective function
//    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
//
//    GRBLinExpr first_objective = lambda_;
//    mip_model_.setObjectiveN(first_objective, 0, 10);
//
//    GRBLinExpr total_keys = 0.0;
//    for (const auto &station : Stations()) {
//        if (station == GroundStation::None) { continue; }
//        for (const auto &interval : StationIntervals(station)) {
//            total_keys += problem_->KeyRate(station, interval.Period(), forecast) * interval.Var();
//        }
//    }
//
//    const double max_keys_sum = std::accumulate(std::cbegin(max_station_keys), std::cend(max_station_keys), 0.0);
//    total_keys_ = mip_model_.addVar(0, max_keys_sum, 0.0, GRB_CONTINUOUS, "total_keys");
//    mip_model_.addConstr(total_keys_ == total_keys);
//    GRBLinExpr second_objective = total_keys_;
//    mip_model_.setObjectiveN(second_objective, 1, 5);
//
//    const auto solver_gap = mip_model_.get(GRB_DoubleParam_MIPGap);
//    auto first_obj_env = mip_model_.getMultiobjEnv(0);
//    first_obj_env.set(GRB_DoubleParam_MIPGap, solver_gap);
//
//    auto second_obj_env = mip_model_.getMultiobjEnv(1);
//    second_obj_env.set(GRB_DoubleParam_MIPGap, solver_gap);
}

void quake::MultiWeekBlockIntervalsMipModel::ReportResults(quake::util::SolverStatus solver_status) {
    BaseMipModel::ReportResults(solver_status);

    std::stringstream msg;

    auto first_obj_env = mip_model_.getMultiobjEnv(0);
    msg << "First objective stop MIP Gap: " << first_obj_env.get(GRB_DoubleParam_MIPGap) << std::endl;

    auto second_obj_env = mip_model_.getMultiobjEnv(1);
    msg << "Second objective stop MIP Gap: " << second_obj_env.get(GRB_DoubleParam_MIPGap) << std::endl;

    msg << "Best solution:" << std::endl
        << " - lambda: " << lambda_.get(GRB_DoubleAttr_X) << std::endl
        << " - total keys: " << total_keys_.get(GRB_DoubleAttr_X) << std::endl;

    // Print number of solutions stored
    auto num_solutions = mip_model_.get(GRB_IntAttr_SolCount);
    msg << "Number of solutions found " << num_solutions << ":" << std::endl;
    msg << " solutions:" << std::endl;
    for (auto solution_index = 0; solution_index < num_solutions; ++solution_index) {
        mip_model_.set(GRB_IntParam_SolutionNumber, solution_index);

        mip_model_.set(GRB_IntParam_ObjNumber, 0);
        const auto lambda = mip_model_.get(GRB_DoubleAttr_ObjNVal);

        mip_model_.set(GRB_IntParam_ObjNumber, 1);
        const auto total_keys = mip_model_.get(GRB_DoubleAttr_ObjNVal);
        msg << " - " << lambda << ", " << total_keys << std::endl;
    }

    LOG(INFO) << msg.str();
}
