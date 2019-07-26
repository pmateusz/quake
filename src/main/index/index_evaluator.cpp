#include "index_evaluator.h"

#include "forecast.h"
#include "solution.h"

quake::IndexEvaluator::IndexEvaluator(const ExtendedProblem &problem)
        : problem_{problem} {}


double quake::IndexEvaluator::Evaluate(const Solution &solution,
                                       const std::function<double(const GroundStation &,
                                                                  const boost::posix_time::time_period &)> &key_rate) const {
    const auto &stations = solution.Stations();

    if (stations.empty()) {
        return 0;
    }

    VLOG(1) << "Keys transferred in solution:";
    double global_traffic_index = std::numeric_limits<double>::max();
    for (const auto &station : solution.Stations()) {
        if (station == GroundStation::None) { continue; }

        double keys_transferred = 0;
        for (const auto &observation_window : solution.ObservationWindows(station)) {
            const auto local_keys_transferred = key_rate(station, observation_window);
            VLOG(1) << station << " " << observation_window << " " << local_keys_transferred;

            keys_transferred += local_keys_transferred;
        }

        double local_traffic_index = keys_transferred / problem_.TransferShare(station);
        if (local_traffic_index == 0) {
            LOG(WARNING) << "Ignoring station " << station << " whose local traffic index is zero";
            continue;
        }
        global_traffic_index = std::min(global_traffic_index, local_traffic_index);
    }
    return global_traffic_index;
}

double quake::IndexEvaluator::operator()(const Solution &solution) const {
    const auto key_rate = [this](const GroundStation &station, const boost::posix_time::time_period &period) -> double {
        return this->problem_.KeyRate(station, period);
    };
    return Evaluate(solution, key_rate);
}

double quake::IndexEvaluator::operator()(const Solution &solution, const Forecast &forecast) const {
    const auto key_rate = [this, &forecast](const GroundStation &station, const boost::posix_time::time_period &period) -> double {
        return this->problem_.KeyRate(station, period, forecast);
    };
    return Evaluate(solution, key_rate);
}

double quake::IndexEvaluator::WorstCase(const Solution &solution, const std::vector<Forecast> &forecasts) const {
    if (forecasts.empty()) {
        return this->operator()(solution);
    }

    double global_traffic_index = std::numeric_limits<double>::max();
    for (const auto &forecast : forecasts) {
        global_traffic_index = std::min(global_traffic_index, this->operator()(solution, forecast));
    }
    return global_traffic_index;

}
