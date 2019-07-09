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

#include <unordered_map>
#include <vector>
#include <iostream>
#include <fstream>
#include <numeric>
#include <tuple>

#include <gtest/gtest.h>
#include <glog/logging.h>
#include <ortools/base/basictypes.h>
#include <ortools/linear_solver/linear_solver.h>
#include <ortools/constraint_solver/constraint_solver.h>
#include <ortools/constraint_solver/routing.h>
#include <ortools/linear_solver/linear_solver.pb.h>


#include <boost/config.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/gregorian_calendar.hpp>
#include <boost/graph/push_relabel_max_flow.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/date_time.hpp>

#include "util/resources.h"
#include "util/hash.h"
#include "util/math.h"
#include "util/logging.h"

#include "problem_generator.h"
#include "problem.h"
#include "sunset_sunrise_reader.h"
#include "transfer_rate_reader.h"
#include "elevation.h"
#include "kepler_elements.h"
#include "minizinc_reader.h"
#include "minizinc_data_model.h"
#include "key_consumption_engine.h"
#include "sunset_sunrise_database.h"
#include "inferred_model.h"
#include "forecast.h"

TEST(ProblemGeneratorTest, CanGenerateEquivalentProblems) {
    // given
    quake::ProblemGenerator generator;

    const auto start_time = boost::posix_time::ptime(boost::gregorian::date(2019, 07, 10),
                                                     boost::posix_time::time_duration());
    const auto end_time = boost::posix_time::ptime(boost::gregorian::date(2019, 07, 12),
                                                   boost::posix_time::time_duration());

    const auto ground_stations = quake::GroundStation::All;
    const boost::posix_time::time_period observation_period(start_time, end_time);

    const auto problem = generator.Create(ground_stations,
                                          start_time,
                                          observation_period);

    const auto extended_problem = generator.CreateExtendedProblem(ground_stations,
                                                                  start_time,
                                                                  observation_period);

    // when
    // then
    EXPECT_EQ(problem.GroundStations(), extended_problem.GroundStations());
    EXPECT_EQ(problem.StartTime(), extended_problem.StartTime());
    EXPECT_EQ(problem.SwitchDuration(), extended_problem.SwitchDuration());
    for (const auto &ground_station : ground_stations) {
        EXPECT_EQ(problem.InitialBuffer(ground_station), extended_problem.InitialBuffer(ground_station));
        EXPECT_EQ(problem.KeyConsumption(ground_station), extended_problem.KeyConsumption(ground_station));
        EXPECT_EQ(problem.TransferWindows(ground_station), extended_problem.TransferWindows(ground_station));
        for (const auto &transfer_window : problem.TransferWindows(ground_station)) {
            for (auto time = transfer_window.begin();
                 time < transfer_window.end(); time += boost::posix_time::seconds(1)) {
                EXPECT_EQ(problem.ElevationAngle(ground_station, time),
                          extended_problem.ElevationAngle(ground_station, time));

                EXPECT_EQ(problem.KeyRate(ground_station, time),
                          extended_problem.KeyRate(ground_station, time));
            }
        }
    }
}

TEST(ResourcesTest, CanFindResourceFile) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};

    // then
    EXPECT_TRUE(boost::filesystem::is_regular_file(resources.SunsetSunriseData(quake::GroundStation::Glasgow)));
}

TEST(CovarianceMatrixText, CanCompyteCovarianceMatrix) {
    boost::numeric::ublas::matrix<double> samples(5, 3);
    samples(0, 0) = 4.0;
    samples(0, 1) = 2.0;
    samples(0, 2) = 0.60;

    samples(1, 0) = 4.2;
    samples(1, 1) = 2.1;
    samples(1, 2) = 0.59;

    samples(2, 0) = 3.9;
    samples(2, 1) = 2.0;
    samples(2, 2) = 0.58;

    samples(3, 0) = 4.3;
    samples(3, 1) = 2.1;
    samples(3, 2) = 0.62;

    samples(4, 0) = 4.1;
    samples(4, 1) = 2.2;
    samples(4, 2) = 0.63;

    boost::numeric::ublas::vector<double> mean = quake::util::mean(samples);
    EXPECT_NEAR(mean(0), 4.10, 1e-6);
    EXPECT_NEAR(mean(1), 2.08, 1e-6);
    EXPECT_NEAR(mean(2), 0.604, 1e-6);

    boost::numeric::ublas::matrix<double> cov_matrix = quake::util::covariance(samples);
    EXPECT_NEAR(cov_matrix(0, 0), 0.025, 1e-6);
    EXPECT_NEAR(cov_matrix(0, 1), 0.0075, 1e-6);
    EXPECT_NEAR(cov_matrix(0, 2), 0.00175, 1e-6);

    EXPECT_NEAR(cov_matrix(1, 0), 0.0075, 1e-6);
    EXPECT_NEAR(cov_matrix(1, 1), 0.0070, 1e-6);
    EXPECT_NEAR(cov_matrix(1, 2), 0.00135, 1e-6);

    EXPECT_NEAR(cov_matrix(2, 0), 0.00175, 1e-6);
    EXPECT_NEAR(cov_matrix(2, 1), 0.00135, 1e-6);
    EXPECT_NEAR(cov_matrix(2, 2), 0.00043, 1e-6);

    boost::numeric::ublas::matrix<double> samples3d(5, 10, 300);
}

TEST(SunsetSunriseReaderTest, CanReadFile) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};
    quake::SunsetSunriseReader reader{quake::util::Resources::DEFAULT_LOCAL_TIME_ZONE};
    auto ground_station = quake::GroundStation::Glasgow;

    // when
    auto results = reader.Read(resources.SunsetSunriseData(ground_station));

    // then
    EXPECT_EQ(results.first, ground_station);

    std::unordered_map<boost::gregorian::date, std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > rise_set_data;
    for (const auto &day : results.second) {
        rise_set_data[day.first.date()] = day;
    }

    using boost::posix_time::ptime;
    using boost::gregorian::date;
    using boost::posix_time::time_duration;
    // expected sunset and sunrise time are converted to UTC time
    std::vector<std::pair<ptime, ptime> > expected_values{
            std::make_pair(ptime(date(2012, 3, 30), time_duration(05, 52, 0)),
                           ptime(date(2012, 3, 30), time_duration(18, 52, 0))),
            std::make_pair(ptime(date(2012, 5, 31), time_duration(03, 41, 0)),
                           ptime(date(2012, 5, 31), time_duration(20, 50, 0))),
            std::make_pair(ptime(date(2030, 12, 31), time_duration(8, 48, 0)),
                           ptime(date(2030, 12, 31), time_duration(15, 53, 0))),
            std::make_pair(ptime(date(2029, 4, 23), time_duration(3, 52, 0)),
                           ptime(date(2029, 4, 23), time_duration(18, 40, 0))),
            std::make_pair(ptime(date(2027, 7, 1), time_duration(2, 36, 0)),
                           ptime(date(2027, 7, 1), time_duration(20, 5, 0)))
    };

    for (const auto &expected_value : expected_values) {
        const auto find_it = rise_set_data.find(expected_value.first.date());
        EXPECT_EQ(expected_value.first, find_it->second.first);
        EXPECT_EQ(expected_value.second, find_it->second.second);
    }
}

TEST(SunsetSunrizeDatabaseTest, CanHandleDaylightSavingTime) {
    // given

    std::vector<quake::GroundStation> ground_stations{quake::GroundStation::London, quake::GroundStation::Glasgow};
    quake::SunsetSunriseDatabase sunset_sunrise_database;

    // when
    sunset_sunrise_database.Load(ground_stations);

    // then
    // switch from winter time to summer time on the 25th of March 2018
    // no jump should be registered because UTC time is returned
    const auto sunset_24march = sunset_sunrise_database.MinSunset(boost::gregorian::date(2018, 3, 24));
    const auto sunset_25march = sunset_sunrise_database.MinSunset(boost::gregorian::date(2018, 3, 25));
    const auto sunset_26march = sunset_sunrise_database.MinSunset(boost::gregorian::date(2018, 3, 26));

    EXPECT_LT(sunset_25march.time_of_day() - sunset_24march.time_of_day(), boost::posix_time::minutes(5));
    EXPECT_LT(sunset_26march.time_of_day() - sunset_25march.time_of_day(), boost::posix_time::minutes(5));
}

TEST(TransferRateReaderTest, CanReadFile) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};
    quake::TransferRateReader reader;

    // when
    auto results = reader.Read(resources.TransferRate(633));

    // then
    for (const auto &tuple : std::vector<std::tuple<double, double, double> >{
            {10, 0.63263203125, 80.9769},
            {45, 30.958359375,  3962.67},
            {90, 67.681953125,  8663.29}}) {
        auto found_value = false;
        for (const auto &row : results) {
            if (std::get<0>(row) == std::get<0>(tuple)) {
                EXPECT_EQ(tuple, row);
                found_value = true;
                break;
            }
        }
        EXPECT_TRUE(found_value) << "Value " << std::get<0>(tuple) << "not found.";
    }
}

static const auto COMPUTATIONAL_ERROR = 0.001;

TEST(TransferRate, CanEstimateTransferRate) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};
    const auto problem_path = resources.GetMiniZincData("week_2018-01-01.dzn");
    const auto cloud_cover_path = resources.GetCloudCoverData("cloud_cover_2018-01-01.csv");

    auto model = quake::InferredModel::Load(problem_path);
    const auto forecast = quake::Forecast::load_csv(cloud_cover_path.c_str());
    model.Apply(forecast);

    std::vector<std::tuple<quake::GroundStation, boost::posix_time::ptime, boost::posix_time::ptime, int64> > test_data{
            {quake::GroundStation::Bristol,
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(21, 29, 30)),
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(23, 6, 35)),
                    515},
            {quake::GroundStation::Manchester,
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(23, 6, 35)
                                             + boost::posix_time::seconds(30)),
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(23, 9, 18)),
                    3648},
            {quake::GroundStation::Bristol,
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 7),
                                             boost::posix_time::time_duration(23, 15, 54)
                                             + boost::posix_time::seconds(30)),
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 8),
                                             boost::posix_time::time_duration(1, 0, 0)),
                    383}
    };

    // when
    // then
    for (const auto &test_instance :test_data) {
        const auto &station = std::get<0>(test_instance);
        const auto &start_time = std::get<1>(test_instance);
        const auto &end_time = std::get<2>(test_instance);
        const auto &keys_transferred = std::get<3>(test_instance);
        EXPECT_EQ(model.TransferredKeys(station, start_time, end_time), keys_transferred);
    }
}

TEST(TransferRate, CanEstimateWeatherAdjustedTransferRate) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};
    const auto problem_path = resources.GetMiniZincData("week_2018-01-01.dzn");
    const auto cloud_cover_path = resources.GetCloudCoverData("cloud_cover_2018-01-01.csv");

    auto model = quake::InferredModel::Load(problem_path);
    const auto forecast = quake::Forecast::load_csv(cloud_cover_path.c_str());
    model.Apply(forecast);

    std::vector<std::tuple<quake::GroundStation, boost::posix_time::ptime, boost::posix_time::ptime, int64> > test_data{
            {quake::GroundStation::Bristol,
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(21, 29, 30)),
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(23, 6, 35)),
                    47936},
            {quake::GroundStation::Manchester,
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(23, 6, 35)
                                             + boost::posix_time::seconds(30)),
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 1),
                                             boost::posix_time::time_duration(23, 9, 18)),
                    91200},
            {quake::GroundStation::Bristol,
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 7),
                                             boost::posix_time::time_duration(23, 15, 54)
                                             + boost::posix_time::seconds(30)),
                    boost::posix_time::ptime(boost::gregorian::date(2018, 1, 8),
                                             boost::posix_time::time_duration(1, 0, 0)),
                    31700}
    };

    // when
    // then
    for (const auto &test_instance :test_data) {
        const auto &station = std::get<0>(test_instance);
        const auto &start_time = std::get<1>(test_instance);
        const auto &end_time = std::get<2>(test_instance);
        const auto &keys_transferred = std::get<3>(test_instance);
        EXPECT_EQ(model.WeatherAdjustedTransferredKeys(station, start_time, end_time), keys_transferred);
    }
}

TEST(ElavationTest, ElevationInMatlabIsConsistentWithSP4) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};
    const auto ground_station = quake::GroundStation::London;
    const auto initial_satellite_position = quake::KeplerElements::DEFAULT;

    boost::posix_time::ptime start_time{{2030, 12, 21}};
    boost::posix_time::ptime end_time{{2030, 12, 22}};
    boost::posix_time::time_period observation_period(start_time, end_time);
    const auto time_step = boost::posix_time::seconds(1);

    // when
    const auto matlab_elevations = quake::GetMatlabElevation(ground_station,
                                                             initial_satellite_position,
                                                             start_time,
                                                             observation_period.length(),
                                                             time_step);

    const auto elevations = quake::GetElevation(ground_station,
                                                initial_satellite_position,
                                                start_time,
                                                observation_period,
                                                time_step);

    // then
    ASSERT_EQ(elevations.size(), matlab_elevations.size());
    const auto size = elevations.size();
    for (auto index = 0; index < size; ++index) {
        ASSERT_NEAR(elevations[0], matlab_elevations[1], COMPUTATIONAL_ERROR);
    }
}

TEST(MiniZincTest, CanReadFile) {
    // given
    quake::util::Resources resources{"~/dev/quake/data"};

    std::ifstream file_stream;
    file_stream.open(resources.GetMiniZincData("2s_1sec_30switch_int.dzn").string());
    ASSERT_TRUE(file_stream.good());

    quake::MiniZincReader<std::ifstream, int64> reader{std::move(file_stream)};
    auto data = reader.Read();
    reader.Close();

    std::vector<std::string> expected_stations{"None", "London", "Glasgow"};
    ASSERT_EQ(data.EnumSet("STATION"), expected_stations);

    const auto &key_rate = data.Array2dOfInt(quake::MiniZincData<int64>::KEY_RATE_REF);
    ASSERT_EQ(key_rate.size(), 3);
    ASSERT_GT(key_rate[2][0], 0);

    quake::MiniZincDataModel<int64, int64> data_model{data};
    for (auto station_index = 0u; station_index < data_model.Stations().size(); ++station_index) {
        const auto cumulative_key_rate = std::accumulate(std::cbegin(data_model.KeyRate().at(station_index)),
                                                         std::cend(data_model.KeyRate().at(station_index)),
                                                         static_cast<int64>(0));
        ASSERT_EQ(cumulative_key_rate,
                  data_model.KeyRateCumulative().at(station_index).at(data_model.Time().size()));
    }
}

TEST(GenerateProblemTest, CanGenerateCorrectTransferRate) {
    // given
    quake::ProblemGenerator generator;
    std::vector<quake::GroundStation> stations{quake::GroundStation::London, quake::GroundStation::Glasgow};
    boost::gregorian::date first_day{2019, 3, 29};
    boost::posix_time::ptime first_time{first_day};
    boost::gregorian::date next_day{2019, 3, 30};
    auto problem = generator.Create(stations,
                                    first_time,
                                    boost::posix_time::time_period{first_time, boost::posix_time::hours(48)});

    std::vector<double> elevation_angle{
            -21.159388669960816,
            -79.2223013749438,
            -3.7686184383170866,
            -58.86224612496999,
            -39.75610900964,
            -31.60263310603082,
            -62.766953498786556,
            -9.212244613231428,
            -70.94363419309853,
            -23.784566695691222,
            -41.561499022097216,
            -62.93826787953066,
            19.0597275339244
    };

    std::vector<double> transfer_rate{
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            4.7208984375
    };

    std::vector<boost::posix_time::ptime> check_points{
            boost::posix_time::ptime(first_day, boost::posix_time::time_duration{18, 0, 0}),
            boost::posix_time::ptime(first_day, boost::posix_time::time_duration{19, 0, 0}),
            boost::posix_time::ptime(first_day, boost::posix_time::time_duration{20, 0, 0}),
            boost::posix_time::ptime(first_day, boost::posix_time::time_duration{21, 0, 0}),
            boost::posix_time::ptime(first_day, boost::posix_time::time_duration{22, 0, 0}),
            boost::posix_time::ptime(first_day, boost::posix_time::time_duration{23, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{0, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{1, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{2, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{3, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{4, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{5, 0, 0}),
            boost::posix_time::ptime(next_day, boost::posix_time::time_duration{6, 0, 0})
    };

    for (auto check_point_index = 0; check_point_index < check_points.size(); ++check_point_index) {
        CHECK_NEAR(elevation_angle[check_point_index],
                   problem.ElevationAngle(quake::GroundStation::London, check_points[check_point_index]),
                   0.001);
    }

    for (auto check_point_index = 0; check_point_index < check_points.size(); ++check_point_index) {
        CHECK_NEAR(transfer_rate[check_point_index],
                   problem.KeyRate(quake::GroundStation::London, check_points[check_point_index]),
                   0.001);
    }
}

TEST(GenerateTransferWeightTest, CanUseLP) {
    // given
    quake::MaxFlowEngine engine;
    std::vector<quake::GroundStation> station{quake::GroundStation::London,
                                              quake::GroundStation::Thurso,
                                              quake::GroundStation::Cambridge,
                                              quake::GroundStation::Birmingham,
                                              quake::GroundStation::Glasgow,
                                              quake::GroundStation::Manchester,
                                              quake::GroundStation::Bristol,
                                              quake::GroundStation::Ipswich,
                                              quake::GroundStation::York};
    std::vector<int64> capacity{8825000, 7933, 124900, 1137100, 621020, 545500, 459300, 133384, 208200};

    // when
    const auto flow_matrix = engine.ComputeFlow(capacity);


    // then
    std::vector<std::vector<int64> > expected_flow_matrix{
            std::vector<int64>{0, 7933, 124900, 1137100, 621020, 545500, 459300, 133384, 208200},
            std::vector<int64>{7933, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{124900, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{1137100, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{621020, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{545500, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{459300, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{133384, 0, 0, 0, 0, 0, 0, 0, 0},
            std::vector<int64>{208200, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    EXPECT_EQ(flow_matrix, expected_flow_matrix);
}

TEST(OrToolsLocalSearch, CanUsePathOperators) {
    operations_research::Solver solver("test_solver");
    const auto node_size = 12;
    const auto vehicle_size = 1;
    CHECK_GT(node_size, 0);

    std::vector<operations_research::IntVar *> next;
    std::vector<operations_research::IntVar *> active;
    std::vector<int64> starts{0};
    std::vector<int64> ends{node_size};
    solver.MakeIntVarArray(node_size, 0, node_size + vehicle_size - 1, "Nexts", &next);
    solver.MakeBoolVarArray(node_size, "Active", &active);

    for (auto next_index = 0; next_index < next.size(); ++next_index) {
        next[next_index]->RemoveValues(starts);
        solver.AddConstraint(solver.MakeIsDifferentCstCt(next[next_index], next_index, active[next_index]));
    }

    for (int i = 0; i < vehicle_size; ++i) {
        for (int j = 0; j < vehicle_size; ++j) {
            if (i != j) {
                next[starts[i]]->RemoveValue(ends[j]);
            }
        }
    }

    solver.AddConstraint(solver.MakeAllDifferent(next, false));
    solver.AddConstraint(solver.MakeNoCycle(next, active));

    operations_research::DecisionBuilder *decision_builder
            = solver.MakeLocalSearchPhase(next,
                                          solver.MakePhase(next,
                                                           operations_research::Solver::IntVarStrategy::CHOOSE_FIRST_UNBOUND,
                                                           operations_research::Solver::IntValueStrategy::ASSIGN_MIN_VALUE),
                                          solver.MakeLocalSearchPhaseParameters(
                                                  solver.ConcatenateOperators(
                                                          {
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::EXCHANGE),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::TWOOPT),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::RELOCATE),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::MAKEACTIVE),
                                                                  solver.MakeOperator(next,
                                                                                      operations_research::Solver::MAKEINACTIVE)
                                                          }),
                                                  nullptr,
                                                  nullptr,
                                                  {solver.MakeVariableDomainFilter()}));

    solver.NewSearch(decision_builder);
    while (solver.NextSolution()) {
        std::ostringstream output_msg;
        output_msg << "Solution " << solver.solutions() << ": ";
        output_msg << 0;
        for (auto node_index = 0; node_index < next.size(); ++node_index) {
            output_msg << ", " << next[node_index];
        }
        LOG(INFO) << output_msg.str();
    }
}

TEST(OrToolsLocalSearch, CanUseLocalSearch) {
    operations_research::Solver solver("test_solver");

    const auto decision_size = 10;
    std::vector<operations_research::IntVar *> decision;
    for (auto decision_index = 0; decision_index < decision_size; ++decision_index) {
        decision.push_back(solver.MakeIntVar(0, decision_size));
    }

    solver.AddConstraint(solver.MakeAllDifferentExcept(decision, 0));

    std::vector<operations_research::IntVar *> cost;
    for (auto decision_index = 0; decision_index < decision_size; ++decision_index) {
        cost.push_back(solver.MakeSquare(decision[decision_index])->Var());
    }

    operations_research::IntVar *total_cost = solver.MakeSum(cost)->Var();
    operations_research::OptimizeVar *objective_value = solver.MakeMinimize(total_cost, 1);
    operations_research::DecisionBuilder *decision_builder = solver.MakeDefaultPhase(decision);
    solver.NewSearch(decision_builder, objective_value);
    while (solver.NextSolution()) {
        std::stringstream output;
        output << "Solution: " << solver.solutions()
               << " " << total_cost->Value()
               << " : " << decision[0]->Value();
        for (auto decision_index = 1; decision_index < decision_size; ++decision_index) {
            output << ", " << decision[decision_index]->Value();
        }
        LOG(INFO) << output.str();
    }
}

TEST(GenerateTransferWeightTest, CanUseMaxFlow) {
    std::unordered_map<quake::GroundStation, int> GroundStationPopulation = {
            {quake::GroundStation::London,     8825000},
            {quake::GroundStation::Thurso,     7933},
            {quake::GroundStation::Cambridge,  124900},
            {quake::GroundStation::Birmingham, 1137100},
            {quake::GroundStation::Glasgow,    621020},
            {quake::GroundStation::Manchester, 545500},
            {quake::GroundStation::Bristol,    459300},
            {quake::GroundStation::Ipswich,    133384},
            {quake::GroundStation::York,       208200}
    };

    // given
    quake::ProblemGenerator generator;
    std::vector<quake::GroundStation> stations{quake::GroundStation::London,
                                               quake::GroundStation::Glasgow,
                                               quake::GroundStation::Thurso//,
//                                               quake::GroundStation::Manchester,
//                                               quake::GroundStation::Birmingham,
//                                               quake::GroundStation::Bristol,
//                                               quake::GroundStation::Ipswich,
//                                               quake::GroundStation::Cambridge,
//                                               quake::GroundStation::York
    };
    boost::gregorian::date first_day{2019, 3, 29};
    boost::gregorian::date next_day{2019, 3, 30};
    boost::posix_time::time_period observation_period{boost::posix_time::ptime{first_day},
                                                      boost::posix_time::hours(48)};
    auto problem = generator.Create(stations, observation_period.begin(), observation_period);
    using Traits = boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>;
    using Graph =  boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
            boost::property<boost::vertex_name_t, std::string>,
            boost::property<boost::edge_capacity_t, long,
                    boost::property<boost::edge_residual_capacity_t, long,
                            boost::property<boost::edge_reverse_t, Traits::edge_descriptor> > > >;

    Graph g;
    auto source_vertex = boost::add_vertex(g);
    auto sink_vertex = boost::add_vertex(g);
    const auto name_property = boost::get(boost::vertex_name, g);
    const auto capacity_property = boost::get(boost::edge_capacity, g);
    const auto reverse_edge_property = boost::get(boost::edge_reverse, g);
    const auto residual_capacity_property = boost::get(boost::edge_residual_capacity, g);

    name_property[source_vertex] = "source";
    name_property[sink_vertex] = "sink";

    std::unordered_map<quake::GroundStation,
            std::tuple<Graph::vertex_descriptor, Graph::vertex_descriptor, Graph::vertex_descriptor> > vertices;
    for (const auto &station : stations) {
        const auto input = boost::add_vertex(g);
        const auto middle = boost::add_vertex(g);
        const auto output = boost::add_vertex(g);
        std::tuple<Graph::vertex_descriptor,
                Graph::vertex_descriptor,
                Graph::vertex_descriptor> station_vertices{input, middle, output};

        name_property[input] = std::string{station.name()} + "_input";
        name_property[middle] = std::string{station.name()} + "_middle";
        name_property[output] = std::string{station.name()} + "_output";

        vertices.emplace(station, station_vertices);
    }

    for (const auto &station_vertices_bundle : vertices) {
        const auto &station_vertices = station_vertices_bundle.second;
        const auto &station = station_vertices_bundle.first;
        const auto transfer_share = static_cast<long>(GroundStationPopulation[station_vertices_bundle.first]);

        Graph::edge_descriptor input_edge, rev_input_edge, output_edge, rev_output_edge;
        bool added_input_edge, added_rev_input_edge, added_output_edge, added_rev_output_edge;

        const auto input_vertex = std::get<0>(station_vertices);
        std::tie(input_edge, added_input_edge) = boost::add_edge(source_vertex, input_vertex, g);
        CHECK(added_input_edge);
        std::tie(rev_input_edge, added_rev_input_edge) = boost::add_edge(input_vertex, source_vertex, g);
        CHECK(added_rev_input_edge);

        capacity_property[input_edge] = transfer_share;
        capacity_property[rev_input_edge] = 0;
        reverse_edge_property[input_edge] = rev_input_edge;
        reverse_edge_property[rev_input_edge] = input_edge;

        const auto output_vertex = std::get<2>(station_vertices);
        std::tie(output_edge, added_output_edge) = boost::add_edge(output_vertex, sink_vertex, g);
        CHECK(added_output_edge);
        std::tie(rev_output_edge, added_rev_output_edge) = boost::add_edge(sink_vertex, output_vertex, g);
        CHECK(added_rev_output_edge);

        capacity_property[output_edge] = transfer_share;
        capacity_property[rev_output_edge] = 0;
        reverse_edge_property[output_edge] = rev_output_edge;
        reverse_edge_property[rev_output_edge] = output_edge;

        for (const auto &other_station_vertices_bundle : vertices) {
            const auto &other_station = other_station_vertices_bundle.first;
            const auto &other_station_vertices = other_station_vertices_bundle.second;
            if (station == other_station) {
                continue;
            }

            const auto other_middle_vertex = std::get<1>(other_station_vertices);
            Graph::edge_descriptor middle_input_edge, rev_middle_input_edge, middle_output_edge, rev_middle_output_edge;
            bool added_middle_input_edge, added_rev_middle_input_edge, added_middle_output_edge, added_rev_middle_output_edge;

            std::tie(middle_input_edge, added_middle_input_edge)
                    = boost::add_edge(input_vertex, other_middle_vertex, g);
            CHECK(added_middle_input_edge);
            std::tie(rev_middle_input_edge, added_rev_middle_input_edge)
                    = boost::add_edge(other_middle_vertex, input_vertex, g);
            CHECK(added_rev_middle_input_edge);

            capacity_property[middle_input_edge] = std::numeric_limits<long>::max();
            capacity_property[rev_middle_input_edge] = 0;
            reverse_edge_property[middle_input_edge] = rev_middle_input_edge;
            reverse_edge_property[rev_middle_input_edge] = middle_input_edge;

            std::tie(middle_output_edge, added_middle_output_edge)
                    = boost::add_edge(other_middle_vertex, output_vertex, g);
            CHECK(added_middle_output_edge);
            std::tie(rev_middle_output_edge, added_rev_middle_output_edge)
                    = boost::add_edge(output_vertex, other_middle_vertex, g);
            CHECK(added_rev_middle_output_edge);

            capacity_property[middle_output_edge] = std::numeric_limits<long>::max();
            capacity_property[rev_middle_output_edge] = 0;
            reverse_edge_property[middle_output_edge] = rev_middle_output_edge;
            reverse_edge_property[rev_middle_output_edge] = middle_output_edge;
        }
    }

    const auto flow = boost::push_relabel_max_flow(g, source_vertex, sink_vertex);

    Graph::vertex_iterator u_iter, u_end;
    Graph::out_edge_iterator ei, e_end;
    for (boost::tie(u_iter, u_end) = boost::vertices(g); u_iter != u_end; ++u_iter) {
        for (boost::tie(ei, e_end) = boost::out_edges(*u_iter, g); ei != e_end; ++ei) {
            if (capacity_property[*ei] > 0) {
                std::cout << "f " << name_property[*u_iter] << " " << name_property[boost::target(*ei, g)] << " "
                          << (capacity_property[*ei] - residual_capacity_property[*ei]) << std::endl;
            }
        }
    }

    LOG(INFO) << flow;

//    long flow;
//
//    property_map<Graph, edge_capacity_t>::type
//            capacity = get(edge_capacity, g);
//    property_map<Graph, edge_reverse_t>::type
//            rev = get(edge_reverse, g);
//    property_map<Graph, edge_residual_capacity_t>::type
//            residual_capacity = get(edge_residual_capacity, g);
//
//    Traits::vertex_descriptor s, t;
//    read_dimacs_max_flow(g, capacity, rev, s, t);
}

int main(int argc, char **argv) {
    quake::util::SetupLogging(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}