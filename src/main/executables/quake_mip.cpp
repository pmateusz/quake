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

#include <vector>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/date_time.hpp>

#include "util/error.h"
#include "util/logging.h"
#include "util/validation.h"

#include "multi_week_block_intervals_mip_model.h"

DEFINE_string(output, "", "The output solution file.");
DEFINE_string(input, "", "The input problem file.");
DEFINE_string(previous_solution, "", "The input file with the previous solution.");
DEFINE_string(time_limit, "", "Time limit for a solver.");
DEFINE_string(time_step, "00:00:15", "Time step for the discretisation scheme.");
DEFINE_string(gap_limit, "", "Optimality gap to stop the MIP solver.");

DEFINE_validator(input, quake::util::validate_input_file);
DEFINE_validator(output, quake::util::validate_output_file);
DEFINE_validator(previous_solution, quake::util::validate_input_file);
DEFINE_validator(time_limit, quake::util::validate_duration);
DEFINE_validator(time_step, quake::util::validate_duration);

struct Arguments {
    Arguments()
            : PreviousSolutionPath{boost::none},
              TimeLimit{boost::none},
              GapLimit{boost::none} {}

    boost::filesystem::path ModelPath;
    boost::filesystem::path OutputSolutionPath;
    boost::optional<boost::filesystem::path> PreviousSolutionPath;
    boost::optional<boost::posix_time::time_duration> TimeLimit;
    boost::optional<double> GapLimit;
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
    CHECK(!FLAGS_time_step.empty()) << "Time step is required";
    CHECK(!FLAGS_output.empty()) << "Solution's output path is required";

    Arguments args;
    args.ModelPath = FLAGS_input;
    args.OutputSolutionPath = FLAGS_output;

    if (!FLAGS_previous_solution.empty()) {
        args.PreviousSolutionPath = FLAGS_previous_solution;
    }

    if (!FLAGS_time_limit.empty()) {
        args.TimeLimit = boost::posix_time::duration_from_string(FLAGS_time_limit);
    }

    if (!FLAGS_gap_limit.empty()) {
        args.GapLimit = std::stod(FLAGS_gap_limit);
    }

    args.TimeStep = boost::posix_time::duration_from_string(FLAGS_time_step);

    return args;
}

int main(int argc, char *argv[]) {
    const auto arguments = SetupLogsAndParseArgs(argc, argv);

    auto model = quake::ExtendedProblem::load_json(arguments.ModelPath);
    if (arguments.PreviousSolutionPath) {
        const auto previous_solution = quake::Solution::load_json(*arguments.PreviousSolutionPath);
        model.OverwriteInitialConditions(previous_solution);
    }

    const auto forecast = model.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);
    quake::MultiWeekBlockIntervalsMipModel mip_model(&model, forecast, arguments.TimeStep);
    const auto solution_opt = mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    if (solution_opt) {
        const auto final_solution = mip_model.UpdateFinalBuffers(solution_opt.get(), forecast);
        quake::util::Save(final_solution, arguments.OutputSolutionPath);
    }

    return EXIT_SUCCESS;
}
