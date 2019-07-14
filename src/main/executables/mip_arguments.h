#ifndef QUAKE_MIP_ARGUMENTS_H
#define QUAKE_MIP_ARGUMENTS_H

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include "util/logging.h"
#include "util/validation.h"

DEFINE_string(problem, "", "The problem file.");
DEFINE_string(solution, "", "The solution output file.");
DEFINE_string(interval_step, "00:00:15", "Interval time step.");
DEFINE_string(time_limit, "", "Time limit for a solver.");
DEFINE_string(gap_limit, "", "Gap between the bound and the objective");

DEFINE_validator(problem, quake::util::validate_input_file);
DEFINE_validator(solution, quake::util::validate_output_file);
DEFINE_validator(time_limit, quake::util::validate_duration);
DEFINE_validator(interval_step, quake::util::validate_duration);

namespace quake {

    struct MipArguments {
        MipArguments()
                : ProblemPath{},
                  SolutionPath{},
                  IntervalStep{},
                  GapLimit{boost::none},
                  TimeLimit{boost::none} {}

        virtual void Fill() {
            CHECK(!FLAGS_problem.empty()) << "The problem input file is required";
            CHECK(!FLAGS_solution.empty()) << "The solution output file is required";
            CHECK(!FLAGS_interval_step.empty()) << "The interval step is required";

            ProblemPath = FLAGS_problem;
            SolutionPath = FLAGS_solution;
            IntervalStep = boost::posix_time::duration_from_string(FLAGS_interval_step);

            if (!FLAGS_time_limit.empty()) {
                TimeLimit = boost::posix_time::duration_from_string(FLAGS_time_limit);
            }

            if (!FLAGS_gap_limit.empty()) {
                GapLimit = std::stod(FLAGS_gap_limit);
            }
        }

        boost::filesystem::path ProblemPath;
        boost::filesystem::path SolutionPath;
        boost::posix_time::time_duration IntervalStep;
        boost::optional<double> GapLimit;
        boost::optional<boost::posix_time::time_duration> TimeLimit;
    };

    template<typename ArgumentsType>
    ArgumentsType SetupLogsAndParseArgs(int argc, char *argv[]) {
        util::SetupLogging(argv[0]);

        CHECK_GE(argc, 1);
        std::stringstream output_msg;
        output_msg << "Program launched with args: " << argv[0];
        for (auto arg_index = 1; arg_index < argc; ++arg_index) {
            output_msg << ' ' << argv[arg_index];
        }
        LOG(INFO) << output_msg.str();

        gflags::ParseCommandLineFlags(&argc, &argv, true);

        ArgumentsType arguments;
        arguments.Fill();
        return arguments;
    }
}

#endif //QUAKE_MIP_ARGUMENTS_H
