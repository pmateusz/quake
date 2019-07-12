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

#include "util/logging.h"
#include "util/validation.h"

#include "index/sample_average_mip_model.h"
#include "index/gaussian_forecast_generator.h"

DEFINE_string(output, "", "The output solution file.");
DEFINE_string(input, "", "The input problem file.");
DEFINE_string(cloud_cover, "", "Forecasts on cloud cover.");
DEFINE_string(time_limit, "", "Time limit for a solver.");
DEFINE_string(time_step, "00:00:15", "Time step for the discretisation scheme.");
DEFINE_string(gap, "", "Gap between the bound and the objective");

DEFINE_validator(input, quake::util::validate_input_file);
DEFINE_validator(output, quake::util::validate_output_file);
DEFINE_validator(cloud_cover, quake::util::validate_input_file);
DEFINE_validator(time_limit, quake::util::validate_duration);
DEFINE_validator(time_step, quake::util::validate_duration);

struct Arguments {
    Arguments() : TimeLimit{boost::none} {}

    boost::filesystem::path ModelPath;
    boost::filesystem::path CloudCoverPath;
    boost::filesystem::path OutputSolutionPath;
    boost::optional<double> Gap;
    boost::optional<boost::posix_time::time_duration> TimeLimit;
    boost::posix_time::time_duration TimeStep;
};

Arguments SetupLogsAndParseArgs(int argc, char *argv[]) {
    quake::util::SetupLogging(argv[0]);

    CHECK_GE(argc, 1);
    std::stringstream output_msg;
    output_msg << "Program launched with args: " << argv[0];
    for (auto arg_index = 1; arg_index < argc; ++arg_index) {
        output_msg << ' ' << argv[arg_index];
    }
    LOG(INFO) << output_msg.str();

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_input.empty()) << "Input problem is required";
    CHECK(!FLAGS_cloud_cover.empty()) << "Cloud cover is required";
    CHECK(!FLAGS_time_step.empty()) << "Time step is required";
    CHECK(!FLAGS_output.empty()) << "Solution's output path is required";

    Arguments args;
    args.ModelPath = FLAGS_input;
    args.CloudCoverPath = FLAGS_cloud_cover;
    args.OutputSolutionPath = FLAGS_output;

    if (!FLAGS_time_limit.empty()) {
        args.TimeLimit = boost::posix_time::duration_from_string(FLAGS_time_limit);
    }

    if (!FLAGS_gap.empty()) {
        args.Gap = std::stod(FLAGS_gap);
    }

    args.TimeStep = boost::posix_time::duration_from_string(FLAGS_time_step);

    return args;
}

int main(int argc, char *argv[]) {
    const auto arguments = SetupLogsAndParseArgs(argc, argv);

    auto model = quake::ExtendedProblem::load_json(arguments.ModelPath);
    const auto primary_forecast = quake::Forecast::load_csv(arguments.CloudCoverPath);

    const auto NUM_SCENARIOS = 128;
    quake::GaussianForecastGenerator gaussian_forecast_generator{1.0};
    auto forecast_scenarios = gaussian_forecast_generator.Generate(primary_forecast, NUM_SCENARIOS);
    forecast_scenarios.insert(forecast_scenarios.begin(), primary_forecast);
    LOG(INFO) << "Generated " << NUM_SCENARIOS << " scenarios";

//    LOG(INFO) << "Computing Worst Case";
//    quake::WorstCaseMipModel worst_case_model(&model, arguments.TimeStep, forecast_scenarios);
//    const auto worst_case_solution_opt = worst_case_model.Solve(arguments.TimeLimit,
//                                                                arguments.Gap,
//                                                                boost::none);
//    CHECK(worst_case_solution_opt) << "Failed to find the worst case solution";
//
//    LOG(INFO) << "Computing Average Case";
//    quake::AverageCaseMipModel average_case_model(&model, arguments.TimeStep, forecast_scenarios);
//    const auto average_case_solution_opt = average_case_model.Solve(arguments.TimeLimit,
//                                                                    arguments.Gap,
//                                                                    boost::none);
//    CHECK(average_case_solution_opt) << "Failed to find the average case solution";
//
//    LOG(INFO) << "Computing Deterministic Case";
    quake::AverageCaseMipModel best_case_model(&model, arguments.TimeStep,
                                               std::vector<quake::Forecast>{primary_forecast});
//    const auto best_case_solution_opt = best_case_model.Solve(arguments.TimeLimit,
//                                                              arguments.Gap,
//                                                              boost::none);
//    CHECK(best_case_solution_opt) << "Failed to find the best case solution";
//
//    LOG(INFO) << "Computing Sample Average Approximation";
//    const auto saa_target_traffic_index = worst_case_model.GetTrafficIndex(*worst_case_solution_opt) * 1.25;
//    quake::SampleAverageMipModel mip_model(&model, arguments.TimeStep, forecast_scenarios, saa_target_traffic_index);
//    const auto saa_solution_opt = mip_model.Solve(arguments.TimeLimit,
//                                                  arguments.Gap,
//                                                  boost::none);
//    CHECK(saa_solution_opt) << "Failed to find the solution for Sample Average Approximation";
//
//    LOG(INFO) << "Worst case: " << best_case_model.GetTrafficIndex(*worst_case_solution_opt);
//    LOG(INFO) << "Average case: " << best_case_model.GetTrafficIndex(*average_case_solution_opt);
//    LOG(INFO) << "Deterministic case: " << best_case_model.GetTrafficIndex(*best_case_solution_opt);
//    LOG(INFO) << "Sample Average Approximation case: " << best_case_model.GetTrafficIndex(*saa_solution_opt);

    LOG(INFO) << "Computing Conditional Value at Risk (epsilon: 0.05)";
//    const auto cvar_target_traffic_index = worst_case_model.GetTrafficIndex(*worst_case_solution_opt) * 1.10;
    const auto cvar_target_traffic_index = 2.64726e+06 * 1.1;
    quake::CVarMipModel cvar_mip_model(&model, arguments.TimeStep, forecast_scenarios, cvar_target_traffic_index, 0.05);
    const auto cvar_solution_opt = cvar_mip_model.Solve(arguments.TimeLimit, arguments.Gap, boost::none);
    CHECK(cvar_solution_opt) << "Failed to find the solution using CVar optimization";

    LOG(INFO) << "Conditional Value at Risk case (epsilon: 0.05): "
              << best_case_model.GetTrafficIndex(*cvar_solution_opt);
    return EXIT_SUCCESS;
}