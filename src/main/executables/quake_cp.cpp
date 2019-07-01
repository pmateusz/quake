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

#include <algorithm>
#include <string>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/exception/diagnostic_information.hpp>

#include <gurobi_c++.h>

#include <ortools/constraint_solver/routing.h>

#include "util/logging.h"
#include "util/resources.h"
#include "util/hash.h"
#include "util/constants.h"
#include "util/validation.h"

#include "legacy/solver_arguments.h"

#include "elevation_command.h"
#include "generate_mini_zinc_data_command.h"
#include "minizinc_reader.h"
#include "problem.h"
#include "problem_generator.h"
#include "solution.h"
#include "solver_cp_command.h"
#include "solver_inventory_cp_command.h"
#include "test_command.h"
#include "inferred_model.h"
#include "forecast.h"

static const std::string ELEVATION_COMMAND{"elevation"};
static const std::string SOLVE_CP_COMMAND{"solve-cp"};
static const std::string SOLVE_INVENTORY_CP_COMMAND{"solve-inventory-cp"};
static const std::string MINIZINC_COMMAND{"minizinc"};
static const std::string TEST_COMMAND{"test"};

static const std::string STATIONS_ARG{"stations"};
static const std::string INPUT_ARG{"input"};
static const std::string OUTPUT_ARG{"output"};
static const std::string DURATION_ARG{"duration"};
static const std::string TIME_STEP_ARG{"time-step"};
static const std::string MIN_JOBS_ARG{"min-jobs"};
static const std::string MAX_JOBS_ARG{"max-jobs"};
static const std::string MIN_DIST_SCALING_FACTOR_ARG{"min-dist-scaling-factor"};
static const std::string MAX_DIST_SCALING_FACTOR_ARG{"max-dist-scaling-factor"};
static const std::string REPEATS_ARG{"repeats"};
static const std::string FAILURE_SCALING_FACTOR_ARG{"failure-scaling-factor"};
static const std::string TIME_LIMIT_ARG{"time-limit"};

static std::runtime_error OnFailedSaveOutput(const boost::filesystem::path &output_file) {
    std::stringstream error_msg;
    error_msg << "Failed to save output in the file: " << output_file << ".";
    return std::runtime_error(error_msg.str());
}

DEFINE_string(command, ELEVATION_COMMAND, "The command to perform. Possible supported are: elevation.");
DEFINE_string(first_day, "2019-03-01", "The start date for which the elevation angle should be computed.");
DEFINE_string(last_day, "2019-03-02", "The end date for which the elevation angle should be computed.");
DEFINE_bool(convert_float_to_int, true, "Use integers instead of floating point numbers.");
DEFINE_bool(print_solutions, true, "Print solutions during search");
DEFINE_int64(time_step, boost::posix_time::minutes(5).total_seconds(),
             "Time step aggregated while compiling the data file");
DEFINE_string(output, "", "The output solution file.");
DEFINE_string(input, "", "The input problem file.");
DEFINE_string(output_initial_guess, "", "The output file for the initial guess solution.");
DEFINE_string(load_initial_guess, "", "The input file for the initial guess solution.");
DEFINE_string(previous_solution, "", "The input file with the previous solution.");
DEFINE_string(stations, "London,Glasgow,Thurso,Manchester,Birmingham,Bristol,Ipswich,Cambridge,York",
              "Locations of ground stations to consider.");
DEFINE_int32(min_jobs, 0, "Minimum number of jobs to consider.");
DEFINE_int32(max_jobs, 0, "Maximum number of jobs to consider.");
DEFINE_int32(job_step, 1, "Incremental change in the number of jobs considered.");
DEFINE_int32(min_dist_scaling_factor, 0, "Minimum scaling factor for distribution to consider.");
DEFINE_int32(max_dist_scaling_factor, 0, "Maximum scaling factor for distribution to consider.");
DEFINE_int32(repeats, 0, "Number of times a randomized computation should be repeated.");
DEFINE_int32(failure_scaling_factor, 0, "Restart the search after a given number of failures.");
DEFINE_string(time_limit, "", "Time limit for a solver.");
DEFINE_string(objective_function, "", "Objective function to use. [cumulative-absolute-error, min-max-relative-error]");
DEFINE_string(cloud_cover, "", "Forecasts on cloud cover.");

static bool validate_command(const char *flagname, const std::string &command) {
    const auto command_to_use = boost::to_lower_copy(command);
    if (command_to_use != ELEVATION_COMMAND
        && command_to_use != MINIZINC_COMMAND
        && command_to_use != SOLVE_CP_COMMAND
        && command_to_use != SOLVE_INVENTORY_CP_COMMAND
        && command_to_use != TEST_COMMAND) {
        LOG(ERROR) << "Invalid command " << command << ".";
        return false;
    }
    return true;
}

template<typename ValueType>
static bool validate_check_positive(const char *flagname, ValueType value) {
    if (value <= 0) {
        std::string flag_name_to_use{flagname};
        boost::to_lower(flag_name_to_use);

        std::string error_msg;
        if (flag_name_to_use == DURATION_ARG) {
            error_msg = "Duration cannot be negative.";
        } else if (flag_name_to_use == TIME_STEP_ARG) {
            error_msg = "Time step cannot be negative.";
        } else {
            LOG(FATAL) << "Validator not configured for flag: " << flagname;
        }

        if (!error_msg.empty()) {
            LOG(ERROR) << error_msg;
        }
        return false;
    }
    return true;
}

template<typename ValueType>
static bool validate_check_non_negative(const char *flagname, ValueType value) {
    if (value < 0) {
        std::string flag_name_to_use{flagname};
        boost::to_lower(flag_name_to_use);

        std::string error_msg;
        if (flag_name_to_use == MIN_JOBS_ARG) {
            error_msg = "Minimum number of jobs cannot be negative.";
        } else if (flag_name_to_use == MAX_JOBS_ARG) {
            error_msg = "Maximum number of jobs cannot be negative";
        } else if (flag_name_to_use == REPEATS_ARG) {
            error_msg = "Number of repeats cannot be negative";
        } else if (flag_name_to_use == FAILURE_SCALING_FACTOR_ARG) {
            error_msg = "Number of failures cannot be negative";
        } else {
            LOG(FATAL) << "Validator not configured for flag: " << flagname;
        }

        if (!error_msg.empty()) {
            LOG(ERROR) << error_msg;
        }
        return false;
    }
    return true;
}

static bool validate_objective_function(const char *flagname, const std::string &objective_function) {
    if (objective_function.empty()) {
        return true;
    }

    const auto opt_objective_value = quake::SolverArguments::parseOrNone(objective_function);
    return static_cast<bool>(opt_objective_value);
}

static std::vector<quake::GroundStation> GetGroundStations(const std::string &input) {
    std::vector<std::string> station_names;
    boost::split(station_names, input, boost::is_any_of(","));

    std::vector<quake::GroundStation> ground_stations;
    for (const auto &station_name : station_names) {
        ground_stations.push_back(quake::GroundStation::FromNameOrNone(station_name));
    }

    return ground_stations;
}

static bool validate_stations(const char *flagname, const std::string &input) {
    std::vector<quake::GroundStation> ground_stations = GetGroundStations(input);
    return std::find(std::cbegin(ground_stations),
                     std::cend(ground_stations),
                     quake::GroundStation::None) == std::cend(ground_stations);
}

static boost::posix_time::time_period parse_time_period(const std::string &first_day_value,
                                                        const std::string &last_day_value) {
    const auto first_date = boost::gregorian::from_string(first_day_value);
    const auto last_date = boost::gregorian::from_string(last_day_value);

    boost::posix_time::ptime first_date_time{first_date};
    boost::posix_time::ptime last_date_time{last_date};
    if (first_date.year() == last_date.year()) {
        static const auto LAST_DAY_DURATION = boost::posix_time::hours(23)
                                              + boost::posix_time::minutes(59)
                                              + boost::posix_time::seconds(59);
        last_date_time += LAST_DAY_DURATION;
    }

    CHECK_LT(first_date_time, last_date_time) << first_date_time << " < " << last_date_time;

    return boost::posix_time::time_period{first_date_time, last_date_time};
}

static quake::SolverArguments get_arguments() {
    quake::SolverArguments args{boost::filesystem::path(FLAGS_input),
                                boost::filesystem::path(FLAGS_output)};

    args.ProblemObjectiveFunction = quake::SolverArguments::parse(FLAGS_objective_function);
    args.SubproblemObjectiveFunction = quake::ObjectiveType::MinTotalAbsoluteError;

    if (!FLAGS_cloud_cover.empty()) {
        args.CloudCoverPath = boost::filesystem::path(FLAGS_cloud_cover);
    }

    if (!FLAGS_load_initial_guess.empty()) {
        args.LoadInitialGuessPath = boost::filesystem::path(FLAGS_load_initial_guess);
    }

    if (!FLAGS_previous_solution.empty()) {
        args.PreviousSolutionPath = boost::filesystem::path(FLAGS_previous_solution);
    }

    if (!FLAGS_output_initial_guess.empty()) {
        args.OutputInitialGuessPath = boost::filesystem::path(FLAGS_output_initial_guess);
    }

    if (!FLAGS_time_limit.empty()) {
        args.TimeLimit = boost::posix_time::duration_from_string(FLAGS_time_limit);
    }

    if (FLAGS_max_dist_scaling_factor != 0) {
        args.MaxScalingFactor = FLAGS_max_dist_scaling_factor;
    }

    if (FLAGS_min_dist_scaling_factor != 0) {
        args.MinScalingFactor = FLAGS_min_dist_scaling_factor;
    }

    if (FLAGS_failure_scaling_factor != 0) {
        args.FailureScalingFactor = FLAGS_failure_scaling_factor;
    }

    if (FLAGS_repeats != 0) {
        args.Repeats = FLAGS_repeats;
    }

    if (FLAGS_min_jobs != 0) {
        args.MinJobs = FLAGS_min_jobs;
    }

    if (FLAGS_max_jobs != 0) {
        args.MaxJobs = FLAGS_max_jobs;
    }

    args.JobsStep = FLAGS_job_step;
    args.PrintSolutions = FLAGS_print_solutions;

    return args;
}

DEFINE_validator(command, validate_command);
DEFINE_validator(first_day, quake::util::validate_date);
DEFINE_validator(last_day, quake::util::validate_date);
DEFINE_validator(time_step, validate_check_positive);
DEFINE_validator(input, quake::util::validate_input_file);
DEFINE_validator(load_initial_guess, quake::util::validate_input_file);
DEFINE_validator(output_initial_guess, quake::util::validate_output_file);
DEFINE_validator(cloud_cover, quake::util::validate_input_file);
DEFINE_validator(previous_solution, quake::util::validate_input_file);
DEFINE_validator(stations, validate_stations);
DEFINE_validator(min_jobs, validate_check_non_negative);
DEFINE_validator(max_jobs, validate_check_non_negative);
DEFINE_validator(job_step, validate_check_positive);
DEFINE_validator(min_dist_scaling_factor, validate_check_non_negative);
DEFINE_validator(max_dist_scaling_factor, validate_check_non_negative);
DEFINE_validator(repeats, validate_check_non_negative);
DEFINE_validator(failure_scaling_factor, validate_check_non_negative);
DEFINE_validator(time_limit, quake::util::validate_duration);
DEFINE_validator(objective_function, validate_objective_function);

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    quake::util::SetupLogging(argv[0]);

    const auto command_to_use = boost::to_lower_copy(FLAGS_command);
    if (command_to_use == ELEVATION_COMMAND) {
        quake::ElevationCommand command{parse_time_period(FLAGS_first_day, FLAGS_last_day),
                                        boost::filesystem::path(FLAGS_output)};
        command.Run();
        return 0;
    } else if (command_to_use == MINIZINC_COMMAND) {
        auto ground_stations = GetGroundStations(FLAGS_stations);
        quake::GenerateMiniZincDataCommand command{parse_time_period(FLAGS_first_day, FLAGS_last_day),
                                                   boost::posix_time::seconds(FLAGS_time_step),
                                                   boost::filesystem::path(FLAGS_output),
                                                   std::move(ground_stations),
                                                   FLAGS_convert_float_to_int};
        command.Run();
        return 0;
    } else if (command_to_use == SOLVE_CP_COMMAND) {
        const auto args = get_arguments();
        quake::SolverCpCommand command{args};
        command.Run();
        return 0;
    } else if (command_to_use == SOLVE_INVENTORY_CP_COMMAND) {
        const auto args = get_arguments();
        quake::SolverInventoryCpCommand command{args};
        command.Run();
        return 0;
    } else if (command_to_use == TEST_COMMAND) {
        auto model = quake::InferredModel::Load("/home/pmateusz/dev/quake/data/simulation/week_2018-01-01.dzn");
        const auto forecast
                = quake::Forecast::load_csv("/home/pmateusz/dev/quake/data/simulation/cloud_cover_2018-01-01.csv");
        model.Apply(forecast);

        const auto station = quake::GroundStation::Bristol;
        const auto date = boost::gregorian::date(2018, 1, 1);
        const auto start_time = boost::posix_time::ptime(date, boost::posix_time::time_duration(21, 29, 30));
        const auto end_time = boost::posix_time::ptime(date, boost::posix_time::time_duration(23, 06, 35));

        LOG(INFO) << station
                  << ' ' << start_time
                  << ' ' << end_time
                  << ' ' << model.TransferredKeys(station, start_time, end_time);

        return 0;
    }

    LOG(ERROR) << "Unknown command: " << FLAGS_command << ".";
    return -1;
}
