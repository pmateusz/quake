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

#include "util/validation.h"
#include "util/logging.h"
#include "forecast.h"
#include "inferred_model.h"
#include "index/gaussian_forecast_generator.h"
#include "index/worst_case_mip_model.h"
#include "index/cross_moment_mip_model.h"

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

    auto model = quake::InferredModel::Load(arguments.ModelPath);
    const auto primary_forecast = quake::Forecast::load_csv(arguments.CloudCoverPath);

    quake::GaussianForecastGenerator gaussian_forecast_generator{1.0};
    auto forecast_scenarios = gaussian_forecast_generator.Generate(primary_forecast, 16);
    forecast_scenarios.insert(forecast_scenarios.begin(), primary_forecast);

    quake::WorstCaseMipModel worst_case_model(&model, arguments.TimeStep, forecast_scenarios);
    const auto worst_case_solution_opt = worst_case_model.Solve(arguments.TimeLimit, arguments.Gap, boost::none);
    CHECK(worst_case_solution_opt) << "Failed to find the worst case solution";

    const auto target_traffic_index = worst_case_model.GetTrafficIndex(*worst_case_solution_opt);
//    const auto target_traffic_index = 2.05602e+06;
    quake::CrossMomentMipModel cross_moment_model(&model, arguments.TimeStep, forecast_scenarios, target_traffic_index);
    const auto cross_moment_solution_opt = cross_moment_model.Solve(arguments.TimeLimit,
                                                                    arguments.Gap,
                                                                    worst_case_solution_opt);
    CHECK(cross_moment_solution_opt) << "Failed to find the cross moment solution";
    LOG(INFO) << cross_moment_model.GetTrafficIndex(*cross_moment_solution_opt);

    return EXIT_SUCCESS;
}