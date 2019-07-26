//
// Copyright 2018 Mateusz Polnik
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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/join.hpp>
#include <index/worst_case_mip_model.h>
#include <index/average_case_mip_model.h>
#include <index/cvar_mip_model.h>
#include <index/index_evaluator.h>

#include "util/logging.h"
#include "util/validation.h"

#include "index/sample_average_mip_model.h"
#include "index/gaussian_forecast_generator.h"
#include "executables/mip_arguments.h"

#include "metadata.h"

DEFINE_int32(num_scenarios, 0, "The number of scenarios to consider.");
DEFINE_string(solution_prefix, "solution", "Output prefix appended to the solution file.");

struct Arguments : public quake::MipArguments {

    Arguments()
            : MipArguments(),
              NumScenarios{0} {}

    void Fill() override {
        MipArguments::Fill();

        CHECK_GE(FLAGS_num_scenarios, 0);
        NumScenarios = FLAGS_num_scenarios;

        CHECK(!FLAGS_solution_prefix.empty());
        SolutionPrefix = FLAGS_solution_prefix;
    }

    int NumScenarios;
    std::string SolutionPrefix;
};

int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<Arguments>(argc, argv);

    const auto problem = quake::ExtendedProblem::load_json(arguments.ProblemPath);
    const auto &weather_forecast = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast);
    const auto &weather_observed = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);
    const auto forecast_scenarios = problem.GetWeatherSamples(quake::ExtendedProblem::WeatherSample::Scenario);
    std::vector<quake::Forecast> forecast_scenarios_to_use;

    if (arguments.NumScenarios > 0) {
        if (arguments.NumScenarios > forecast_scenarios.size()) {
            LOG(WARNING) << "Requested more scenarios than the problem contains " << arguments.NumScenarios
                         << " v.s. " << forecast_scenarios.size()
                         << ". The simulation will effectively use " << forecast_scenarios.size() << " scenarios.";
            forecast_scenarios_to_use = forecast_scenarios;
        } else {
            const auto scenario_begin_it = std::cbegin(forecast_scenarios);
            const auto scenario_end_it = scenario_begin_it + arguments.NumScenarios;
            std::copy(scenario_begin_it, scenario_end_it, std::back_inserter(forecast_scenarios_to_use));
        }
    }

    // compute saa index over generated scenarios
    const auto target_traffic_index = evaluator(*observed_solution_opt) * 0.15;
//    LOG(INFO) << "Computing Sample Average Approximation with the target index of " << target_traffic_index;
//    quake::SampleAverageMipModel mip_model(&problem, arguments.IntervalStep, forecast_scenarios_to_use, target_traffic_index);
//    const auto saa_solution_opt = mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
//    CHECK(saa_solution_opt) << "Failed to find the solution for Sample Average Approximation";
//    LOG(INFO) << "Sample Average Approximation  In-Sample Traffic Index: " << evaluator(*saa_solution_opt, weather_forecast);
//    LOG(INFO) << "Sample Average Approximation Out-of-Sample Traffic Index: " << evaluator(*saa_solution_opt, weather_observed);

    quake::CVarMipModel cvar_mip_model(&problem, arguments.IntervalStep, forecast_scenarios_to_use, target_traffic_index, 0.05);
    const auto cvar_solution_opt = cvar_mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    CHECK(cvar_solution_opt) << "Failed to find the solution using CVar optimization";
    LOG(INFO) << "Conditional Value at Risk In-Sample Traffic Index: " << evaluator(*cvar_solution_opt, weather_forecast);
    LOG(INFO) << "Conditional Value at Risk Out-of-Sample Traffic Index: " << evaluator(*cvar_solution_opt, weather_observed);

//    LOG(INFO) << "Computing Worst Case";
//    quake::WorstCaseMipModel worst_case_model(&problem, arguments.IntervalStep, forecast_scenarios);

//    CHECK(worst_case_solution_opt) << "Failed to find the worst case solution";

//    LOG(INFO) << "Computing Average Case";
//    quake::AverageCaseMipModel average_case_model(&model, arguments.TimeStep, forecast_scenarios);
//    const auto average_case_solution_opt = average_case_model.Solve(arguments.TimeLimit,
//                                                                    arguments.Gap,
//                                                                    boost::none);
//    CHECK(average_case_solution_opt) << "Failed to find the average case solution";
//
//    LOG(INFO) << "Computing Deterministic Case";
//    quake::AverageCaseMipModel best_case_model(&model, arguments.TimeStep,
//                                               std::vector<quake::Forecast>{primary_forecast});
//    const auto best_case_solution_opt = best_case_model.Solve(arguments.TimeLimit,
//                                                              arguments.Gap,
//                                                              boost::none);
//    CHECK(best_case_solution_opt) << "Failed to find the best case solution";
//


//    const auto THRESHOLD = 1.25;
//    const auto worst_cast_traffic_index = evaluator(*worst_case_solution_opt, weather_forecast);
//    const auto target_traffic_index = worst_cast_traffic_index * THRESHOLD;





//    LOG(INFO) << "Worst case: " << best_case_model.GetTrafficIndex(*worst_case_solution_opt);
//    LOG(INFO) << "Average case: " << best_case_model.GetTrafficIndex(*average_case_solution_opt);
//    LOG(INFO) << "Deterministic case: " << best_case_model.GetTrafficIndex(*best_case_solution_opt);
//    LOG(INFO) << "Sample Average Approximation case: " << best_case_model.GetTrafficIndex(*saa_solution_opt);

//    LOG(INFO) << "Computing Conditional Value at Risk (epsilon: 0.05)";
//    const auto cvar_target_traffic_index = worst_case_model.GetTrafficIndex(*worst_case_solution_opt) * 1.10;
//    const auto cvar_target_traffic_index = 2.64726e+06 * 1.1;


//    LOG(INFO) << "Conditional Value at Risk case (epsilon: 0.05): "
//              << best_case_model.GetTrafficIndex(*cvar_solution_opt);
    return EXIT_SUCCESS;
}