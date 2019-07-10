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

#include "util/validation.h"
#include "util/logging.h"

#include "extended_problem.h"

DEFINE_string(problem_file, "", "The problem file.");

DEFINE_validator(problem_file, quake::util::validate_input_file);

struct Arguments {
    boost::filesystem::path ProblemPath;
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

    CHECK(!FLAGS_problem_file.empty()) << "Problem file is required";

    Arguments args;
    args.ProblemPath = FLAGS_problem_file;

    return args;
}

int main(int argc, char *argv[]) {
    const auto arguments = SetupLogsAndParseArgs(argc, argv);

    std::ifstream input_stream;
    input_stream.open(arguments.ProblemPath.string(), std::ifstream::in);

    LOG_IF(FATAL, !input_stream.is_open()) << "Failed to open file " << arguments.ProblemPath;

    nlohmann::json json_object;
    input_stream >> json_object;
    input_stream.close();

    const auto problem = json_object.get<quake::ExtendedProblem>();
    const auto rounded_problem = problem.Round(2);

    return EXIT_SUCCESS;
}