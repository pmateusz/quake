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

DEFINE_string(problem, "", "Problem file.");
DEFINE_string(interval_step, "00:00:15", "Interval time step.");
DEFINE_string(time_limit, "", "Time limit for a solver.");
DEFINE_string(gap_limit, "", "Gap between the bound and the objective");

DEFINE_string(output, "", "Solution file.");
DEFINE_double(target_traffic_index, 1, "Target traffic index.");
DEFINE_int32(num_scenarios, 1, "Number of scenarios to consider.");

DEFINE_validator(problem, quake::util::validate_input_file);
DEFINE_validator(output, quake::util::validate_output_file);
DEFINE_validator(time_limit, quake::util::validate_duration);
DEFINE_validator(interval_step, quake::util::validate_duration);


namespace quake {

    struct MipArguments {
        MipArguments()
                : ProblemPath{},
                  IntervalStep{},
                  GapLimit{boost::none},
                  TimeLimit{boost::none} {}

        virtual void Fill() {
            CHECK(!FLAGS_problem.empty()) << "The problem input file is required";
            CHECK(!FLAGS_interval_step.empty()) << "The interval step is required";

            ProblemPath = FLAGS_problem;
            IntervalStep = boost::posix_time::duration_from_string(FLAGS_interval_step);

            if (!FLAGS_time_limit.empty()) {
                TimeLimit = boost::posix_time::duration_from_string(FLAGS_time_limit);
            }

            if (!FLAGS_gap_limit.empty()) {
                GapLimit = std::stod(FLAGS_gap_limit);
            }
        }

        boost::filesystem::path ProblemPath;
        boost::posix_time::time_duration IntervalStep;
        boost::optional<double> GapLimit;
        boost::optional<boost::posix_time::time_duration> TimeLimit;
    };


    struct ScenarioIndexMipArguments : public quake::MipArguments {

        ScenarioIndexMipArguments()
                : MipArguments(),
                  TargetTrafficIndex{0.0},
                  NumScenarios{0},
                  SolutionFile{""} {}

        void Fill() override {
            MipArguments::Fill();

            CHECK_GT(FLAGS_target_traffic_index, 0.0);
            TargetTrafficIndex = FLAGS_target_traffic_index;

            CHECK_GT(FLAGS_num_scenarios, 0);
            NumScenarios = FLAGS_num_scenarios;

            CHECK(!FLAGS_output.empty());
            SolutionFile = FLAGS_output;
        }

        double TargetTrafficIndex;
        int NumScenarios;
        boost::filesystem::path SolutionFile;
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
