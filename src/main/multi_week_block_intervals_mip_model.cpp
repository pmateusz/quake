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

    // split week into periods ending every Sunday
    boost::posix_time::time_period observation_period = problem_->ObservationPeriod();
    std::vector<boost::posix_time::time_period> milestone_periods;
    boost::posix_time::ptime current_week_start = observation_period.begin();
    while (current_week_start < observation_period.end()) {
        // every Sunday is the end of the week instead of a constant day period
        const auto next_week_days = 7 - current_week_start.date().day_of_week();
        boost::posix_time::ptime next_week_start{current_week_start.date() + boost::gregorian::days(next_week_days),
                                                 current_week_start.time_of_day()};

        next_week_start = std::min(observation_period.end(), next_week_start);

        const auto period = boost::posix_time::time_period{current_week_start, next_week_start};
        milestone_periods.emplace_back(period);

        current_week_start = next_week_start;
    }
    const auto num_milestones = milestone_periods.size();

    // compute number of keys delivered every milestone
    std::vector<std::vector<GRBLinExpr>> keys_transferred_by_station_by_milestone(Stations().size(), std::vector<GRBLinExpr>(num_milestones));
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);

        auto milestone_index = 0;
        for (const auto &interval : StationIntervals(station)) {
            while (milestone_index < num_milestones && milestone_periods.at(milestone_index).is_before(interval.Period().begin())) {
                ++milestone_index;
            }

            CHECK(milestone_periods.at(milestone_index).contains(interval.Period()));

            keys_transferred_by_station_by_milestone.at(station_index).at(milestone_index)
                    += problem_->KeyRate(station, interval.Period(), forecast) * interval.Var();
        }
    }

    CHECK_GE(num_milestones, 1);
    for (const auto &initial_buffer_entry : initial_buffer_to_use) {
        const auto station_index = Index(initial_buffer_entry.first);
        keys_transferred_by_station_by_milestone.at(station_index).at(0) += initial_buffer_entry.second;
    }

    lambda_ = mip_model_.addVar(0, max_lambda, 0, GRB_CONTINUOUS);
    for (const auto &station : ObservableStations()) {
        const auto station_index = Index(station);

        GRBLinExpr final_key_buffer = 0;
        for (std::size_t milestone_index = 0; milestone_index < num_milestones; ++milestone_index) {
            final_key_buffer += keys_transferred_by_station_by_milestone.at(station_index).at(milestone_index);

            mip_model_.addConstr(lambda_ <= final_key_buffer / TransferShare(station));
        }
    }

    // objective function
    GRBLinExpr objective = lambda_;
    mip_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
    mip_model_.setObjective(objective);

    LOG(INFO) << "Model Configuration Finished";
}

void quake::MultiWeekBlockIntervalsMipModel::ReportResults(quake::util::SolverStatus solver_status) {
    BaseMipModel::ReportResults(solver_status);

    std::stringstream msg;
    msg << "Objective stop MIP Gap limit: " << mip_model_.get(GRB_DoubleParam_MIPGap);
    msg << "Best solution:" << std::endl
        << " - lambda: " << lambda_.get(GRB_DoubleAttr_X) << std::endl;

    auto num_solutions = mip_model_.get(GRB_IntAttr_SolCount);
    msg << "Number of solutions found " << num_solutions << ":" << std::endl;
    msg << " solutions:" << std::endl;
    for (auto solution_index = 0; solution_index < num_solutions; ++solution_index) {
        mip_model_.set(GRB_IntParam_SolutionNumber, solution_index);
        const auto lambda = mip_model_.get(GRB_DoubleAttr_ObjVal);
        msg << " - " << lambda << std::endl;
    }

    LOG(INFO) << msg.str();
}
