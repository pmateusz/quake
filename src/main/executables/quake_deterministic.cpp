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

#include <cstdlib>
#include <string>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include "extended_problem.h"
#include "index/worst_case_mip_model.h"
#include "mip_arguments.h"
#include "metadata.h"

boost::optional<quake::ExtendedProblem::WeatherSample> TryParseWeatherSample(const std::string &value) {
    const auto value_to_use = boost::algorithm::to_lower_copy(value);

    if (value_to_use == "forecast") {
        return quake::ExtendedProblem::WeatherSample::Forecast;
    } else if (value_to_use == "real") {
        return quake::ExtendedProblem::WeatherSample::Real;
    } else if (value_to_use == "scenario") {
        return quake::ExtendedProblem::WeatherSample::Scenario;
    } else if (value_to_use == "none") {
        return quake::ExtendedProblem::WeatherSample::None;
    }

    return boost::none;
}


inline bool ValidateSample(const char *flag_name, const std::string &value) {
    const auto value_to_use = boost::algorithm::to_lower_copy(value);
    const auto weather_sample = TryParseWeatherSample(value_to_use);

    const auto validation_result = weather_sample == quake::ExtendedProblem::WeatherSample::Forecast
                                   || weather_sample == quake::ExtendedProblem::WeatherSample::Real;
    if (validation_result) { return true; }

    LOG(ERROR) << flag_name << " can be set to either 'real' or 'forecast'";
    return false;
}

DEFINE_string(sample, "forecast", "Sample to use as the forecast.");
DEFINE_validator(sample, ValidateSample);

struct Arguments : public quake::MultiStageArguments {

    Arguments()
            : MultiStageArguments(),
              Sample{quake::ExtendedProblem::WeatherSample::None} {}

    void Fill() override {
        MultiStageArguments::Fill();

        Sample = *TryParseWeatherSample(FLAGS_sample);
    }

    quake::ExtendedProblem::WeatherSample Sample;
};


boost::optional<quake::Solution> Solve(const Arguments &args, const quake::ExtendedProblem &problem, const quake::Forecast &forecast) {
    quake::WorstCaseMipModel mip_model{&problem, args.IntervalStep, {forecast}};
    return mip_model.Solve(args.TimeLimit, args.GapLimit, boost::none);
}

// TODO: assess the solution using monte carlo
// TODO: assess the solution using real weather
int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<Arguments>(argc, argv);

    std::vector<quake::ExtendedProblem> problems;
    for (const auto &file_path : arguments.ProblemPaths) {
        problems.emplace_back(quake::ExtendedProblem::load_json(file_path));
    }
    CHECK(!problems.empty());

    std::sort(problems.begin(), problems.end(), [](const quake::ExtendedProblem &left, const quake::ExtendedProblem &right) -> bool {
        return left.ObservationPeriod().begin() < right.ObservationPeriod().begin();
    });

    const auto &master_problem = problems.front();
    const auto master_observation_period = master_problem.ObservationPeriod();
    const auto &master_real_forecast = master_problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);

    std::vector<quake::ExtendedProblem> trimmed_problems{master_problem};
    const auto problem_it_end = problems.end();
    for (auto problem_it = problems.begin() + 1; problem_it != problem_it_end; ++problem_it) {
        boost::posix_time::time_period observation_period{problem_it->ObservationPeriod().begin(), master_observation_period.end()};

        const auto trimmed_problem = problem_it->Trim(observation_period);
        // check if real forecast agree with the trimmed and master problem
        const auto &trimmed_real_forecast = trimmed_problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);

//        for (const auto &trimmed_entry : trimmed_real_forecast.Index()) {
//            auto time_point = trimmed_entry.second.Period().begin();
//            while (time_point < trimmed_entry.second.Period().end()) {
//                CHECK_EQ(master_real_forecast.GetCloudCover(trimmed_entry.first, time_point),
//                         trimmed_real_forecast.GetCloudCover(trimmed_entry.first, time_point));
//                time_point += trimmed_entry.second.UpdateFrequency();
//            }
//        }

        trimmed_problems.emplace_back(trimmed_problem);
    }

    std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > observations;
    for (decltype(trimmed_problems.size()) problem_index = 0; problem_index < trimmed_problems.size(); ++problem_index) {
        auto &problem = trimmed_problems.at(problem_index);

        std::unordered_map<quake::GroundStation, int64> station_buffers;
        for (const auto &station : problem.Stations()) {
            station_buffers[station] = problem.InitialBuffer(station);

            if (observations.find(station) == std::end(observations)) {
                continue;
            }

            for (const auto &observation: observations[station]) {
                station_buffers[station] += problem.KeyRate(station, observation, master_real_forecast);
            }
        }
        quake::Solution initial_solution{station_buffers};
        problem.OverwriteInitialConditions(initial_solution);

        auto decision_period = problem.ObservationPeriod();
        const auto next_problem_index = problem_index + 1;
        if (next_problem_index < trimmed_problems.size()) {
            const auto &next_problem = trimmed_problems.at(next_problem_index);
            decision_period = boost::posix_time::time_period{decision_period.begin(), next_problem.ObservationPeriod().begin()};
        }

        const auto &forecast = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast);
        boost::filesystem::path solution_file_path = (boost::format("forecast_%1%.json") % problem.ObservationPeriod().begin().date()).str();
        const auto solution_opt = Solve(arguments, problem, forecast);
        for (const auto &local_observation_group : solution_opt->Observations()) {
            for (const auto &local_observation : local_observation_group.second) {
                CHECK(!decision_period.intersects(local_observation) || decision_period.contains(local_observation));
                if (decision_period.contains(local_observation)) {
                    observations[local_observation_group.first].emplace_back(local_observation);
                }
            }
        }
    }

    std::unordered_map<quake::GroundStation, int64> final_key_buffers;
    for (const auto &station : master_problem.Stations()) {
        final_key_buffers[station] = master_problem.InitialBuffer(station);
    }
    for (const auto &observation_group : observations) {
        for (const auto &observation : observation_group.second) {
            final_key_buffers[observation_group.first] += master_problem.KeyRate(observation_group.first, observation, master_real_forecast);
        }
    }

    quake::Solution solution{master_problem.GetMetadata(), observations, final_key_buffers};
    LOG(INFO) << master_problem.GetTrafficIndex(solution.Observations(), master_real_forecast);

    quake::util::Save(solution, arguments.SolutionFile);

//    {
//        LOG(INFO) << "Solving model with weather forecast";
//        const auto &forecast = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast);
//        boost::filesystem::path solution_file_path = (boost::format("%1%_deterministic_forecast.json") % arguments.SolutionPrefix).str();
//        Solve(arguments, problem, forecast, quake::Metadata::SolutionType::Test, solution_file_path);
//    }
//
//    {
//        LOG(INFO) << "Solving model with weather forecast";
//        const auto &real_weather = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);
//        boost::filesystem::path solution_file_path = (boost::format("%1%_deterministic_real.json") % arguments.SolutionPrefix).str();
//        Solve(arguments, problem, real_weather, quake::Metadata::SolutionType::Reference, solution_file_path);
//    }

    return EXIT_SUCCESS;
}