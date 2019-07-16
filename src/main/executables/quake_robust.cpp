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
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <nlohmann/json.hpp>
#include <index/robust_average_mip_model.h>

#include "util/validation.h"
#include "util/logging.h"

#include "extended_problem.h"
#include "index/robust_index_mip_model.h"
#include "index/worst_case_mip_model.h"
#include "index/index_evaluator.h"
#include "executables/mip_arguments.h"


int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<quake::MipArguments>(argc, argv);
    const auto problem = quake::ExtendedProblem::load_json(arguments.ProblemPath);
    const auto &weather_forecast = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast);
    const auto &weather_observed = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);

    LOG(INFO) << "Computing Worst Case";
    quake::WorstCaseMipModel worst_case_model(&problem, arguments.IntervalStep, {weather_forecast});
    const auto worst_case_solution_opt = worst_case_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    CHECK(worst_case_solution_opt) << "Failed to find the worst case solution";
//
    quake::IndexEvaluator evaluator{problem};
    const auto worst_case_traffic_index = evaluator(*worst_case_solution_opt, weather_forecast);
    LOG(INFO) << "Worst Case In-Sample Traffic Index: " << evaluator(*worst_case_solution_opt, weather_forecast);
    LOG(INFO) << "Worst Cast Out-of-Sample Traffic Index: " << evaluator(*worst_case_solution_opt, weather_observed);

//    const auto THRESHOLD = 1.10;
//    const auto target_traffic_index = worst_cast_traffic_index * THRESHOLD;
//    LOG(INFO) << "Computing Distributionally Robust Essential Riskiness Index with the target index of " << target_traffic_index
//              << " at threshold of " << THRESHOLD;

//    const auto worst_case_traffic_index = 8519;
    quake::RobustIndexMipModel robust_mip_model{&problem, arguments.IntervalStep, 0.5 * worst_case_traffic_index};
//    quake::RobustAverageMipModel robust_mip_model{&problem, arguments.IntervalStep};
    const auto solution_opt = robust_mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    CHECK(solution_opt);
    LOG(INFO) << "Distributionally Robust In-Sample Traffic Index: " << evaluator(*solution_opt, weather_forecast);
    LOG(INFO) << "Distributionally Robust Out-of-Sample Traffic Index: " << evaluator(*solution_opt, weather_observed);

    return EXIT_SUCCESS;
}