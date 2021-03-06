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

#include <regex>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <kepler_elements.h>

#include "util/validation.h"
#include "util/logging.h"
#include "problem_generator.h"
#include "extended_problem.h"

DEFINE_string(from, "2019-03-01", "The start date for which the elevation angle should be computed.");
DEFINE_string(to, "2019-03-02", "The end date for which the elevation angle should be computed.");
DEFINE_string(initial_epoch, "", "Initial epoch. Should be set to the same value for problems with a rolling horizon.");
DEFINE_string(output, "", "The output solution file.");
DEFINE_string(stations, "", "Station names separated by a non-alpha character");
DEFINE_double(altitude, quake::KeplerElements::DEFAULT_ALTITUDE, "Altitude.");
DEFINE_double(inclination, quake::KeplerElements::DEFAULT_INCLINATION, "Inclination.");
DEFINE_double(raan, quake::KeplerElements::DEFAULT_RAAN, "RAAN");

DEFINE_validator(output, quake::util::validate_output_file);
DEFINE_validator(from, quake::util::validate_date);
DEFINE_validator(to, quake::util::validate_date);
DEFINE_validator(initial_epoch, quake::util::validate_datetime);

struct Arguments {
    Arguments()
            : Altitude(quake::KeplerElements::DEFAULT_ALTITUDE),
              Inclination(quake::KeplerElements::DEFAULT_INCLINATION),
              RAAN(quake::KeplerElements::DEFAULT_RAAN),
              ProblemOutputPath(""),
              ObservationTime(boost::posix_time::time_period(boost::posix_time::ptime(),
                                                             boost::posix_time::seconds(0))),
              InitialEpoch(ObservationTime.begin()) {}

    double Altitude;
    double Inclination;
    double RAAN;

    boost::filesystem::path ProblemOutputPath;
    boost::posix_time::time_period ObservationTime;
    boost::posix_time::ptime InitialEpoch;
    std::vector<quake::GroundStation> Stations;
};

boost::posix_time::ptime ParseDateIgnoreTime(const std::string &value) {
    static const auto ZERO_DURATION = boost::posix_time::seconds(0);
    return {boost::gregorian::from_simple_string(value), ZERO_DURATION};
}

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

    CHECK(!FLAGS_to.empty()) << "to argument is required";
    CHECK(!FLAGS_from.empty()) << "from argument is required";
    CHECK(!FLAGS_output.empty()) << "output argument is required";

    Arguments args;
    args.Altitude = FLAGS_altitude;
    args.Inclination = FLAGS_inclination;
    args.RAAN = FLAGS_raan;
    args.ProblemOutputPath = FLAGS_output;

    std::vector<quake::GroundStation> stations;
    std::regex split_pattern{"[^\\w]+"};
    std::sregex_token_iterator end;
    for (std::sregex_token_iterator iterator(FLAGS_stations.begin(), FLAGS_stations.end(), split_pattern, -1); iterator != end; ++iterator) {
        stations.emplace_back(quake::GroundStation::FromName(*iterator));
    }

    if (stations.empty()) {
        stations = quake::GroundStation::All;
    }
    args.Stations = stations;

    const auto start_time = ParseDateIgnoreTime(FLAGS_from);
    const auto end_time = ParseDateIgnoreTime(FLAGS_to);
    CHECK_LE(start_time, end_time);

    boost::posix_time::ptime initial_epoch = start_time;
    if (!FLAGS_initial_epoch.empty()) {
        initial_epoch = boost::posix_time::time_from_string(FLAGS_initial_epoch);
    }

    args.ObservationTime = boost::posix_time::time_period(start_time + boost::posix_time::hours(12),
                                                          end_time + boost::posix_time::hours(12));
    args.InitialEpoch = initial_epoch;
    return args;
}

void Save(const Arguments &arguments, const quake::ExtendedProblem &problem) {
    nlohmann::json body = problem;

    std::ofstream output_stream;
    output_stream.open(arguments.ProblemOutputPath.string(), std::ofstream::out);
    CHECK(output_stream.is_open());

    output_stream << body;
    output_stream.close();
}

int main(int argc, char *argv[]) {
    const auto arguments = SetupLogsAndParseArgs(argc, argv);

    quake::ProblemGenerator generator;
    const auto problem = generator.CreateExtendedProblem(arguments.Stations,
                                                         arguments.Altitude,
                                                         arguments.Inclination,
                                                         arguments.RAAN,
                                                         arguments.InitialEpoch,
                                                         arguments.ObservationTime);
    Save(arguments, problem.Round(3));
    return EXIT_SUCCESS;
}
