#ifndef QUAKE_MIP_ARGUMENTS_H
#define QUAKE_MIP_ARGUMENTS_H

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include "util/logging.h"
#include "util/validation.h"

inline std::vector<boost::filesystem::path> ParsePaths(const std::string &value) {
    std::vector<std::string> raw_file_paths;
    boost::split(raw_file_paths, value, boost::is_any_of(":"));

    std::vector<boost::filesystem::path> file_paths;
    file_paths.reserve(raw_file_paths.size());

    for (const auto &raw_file_path : raw_file_paths) {
        file_paths.emplace_back(raw_file_path);
    }

    return file_paths;
}

inline bool ValidateProblemFiles(const char *flag_name, const std::string &value) {
    const auto file_paths = ParsePaths(value);

    for (const auto &file_path : file_paths) {
        if (!boost::filesystem::exists(file_path)) {
            LOG(ERROR) << "File '" << file_path << "' does not exist.";
            return false;
        }
    }

    return true;
}

DEFINE_string(problem, "", "Problem file.");
DEFINE_string(problems, "", "Collection of problem files split by colon.");
DEFINE_string(interval_step, "00:00:15", "Interval time step.");
DEFINE_string(time_limit, "", "Time limit for a solver.");
DEFINE_string(gap_limit, "", "Gap between the bound and the objective");

DEFINE_string(output, "", "Solution file.");
DEFINE_double(target_traffic_index, 1, "Target traffic index.");
DEFINE_int32(num_scenarios, 1, "Number of scenarios to consider.");

DEFINE_validator(problem, quake::util::validate_input_file);
DEFINE_validator(problems, ValidateProblemFiles);
DEFINE_validator(output, quake::util::validate_output_file);
DEFINE_validator(time_limit, quake::util::validate_duration);
DEFINE_validator(interval_step, quake::util::validate_duration);


namespace quake {

    struct Arguments {
        Arguments()
                : IntervalStep{},
                  SolutionFile{},
                  GapLimit{boost::none},
                  TimeLimit{boost::none} {}

        virtual void Fill() {
            CHECK(!FLAGS_interval_step.empty()) << "The interval step is required";
            IntervalStep = boost::posix_time::duration_from_string(FLAGS_interval_step);

            CHECK(!FLAGS_output.empty()) << "The solution file is required";
            SolutionFile = FLAGS_output;

            if (!FLAGS_time_limit.empty()) {
                TimeLimit = boost::posix_time::duration_from_string(FLAGS_time_limit);
            }

            if (!FLAGS_gap_limit.empty()) {
                GapLimit = std::stod(FLAGS_gap_limit);
            }
        }

        boost::posix_time::time_duration IntervalStep;
        boost::optional<double> GapLimit;
        boost::optional<boost::posix_time::time_duration> TimeLimit;

        boost::filesystem::path SolutionFile;
    };

    struct SingleStageArguments : Arguments {
        SingleStageArguments()
                : Arguments(),
                  ProblemPath{} {}

        virtual void Fill() {
            CHECK(!FLAGS_problems.empty()) << "The problem input file is required";

            ProblemPath = FLAGS_problem;
        }

        boost::filesystem::path ProblemPath;
    };

    struct MultiStageArguments : Arguments {

        virtual void Fill() {
            Arguments::Fill();

            CHECK(!FLAGS_problems.empty()) << "The problem files are required";

            ProblemPaths = ParsePaths(FLAGS_problems);
        }

        std::vector<boost::filesystem::path> ProblemPaths;
    };

    struct ScenarioIndexMipArguments : public SingleStageArguments {

        ScenarioIndexMipArguments()
                : SingleStageArguments(),
                  TargetTrafficIndex{0.0},
                  NumScenarios{0} {}

        void Fill() override {
            Arguments::Fill();

            CHECK_GT(FLAGS_target_traffic_index, 0.0);
            TargetTrafficIndex = FLAGS_target_traffic_index;

            CHECK_GT(FLAGS_num_scenarios, 0);
            NumScenarios = FLAGS_num_scenarios;
        }

        double TargetTrafficIndex;
        int NumScenarios;
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
