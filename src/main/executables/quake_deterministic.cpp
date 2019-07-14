//
// Copyright 2019 Mateusz Polnik
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

#include "extended_problem.h"
#include "index/worst_case_mip_model.h"
#include "index/index_evaluator.h"
#include "mip_arguments.h"


int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<quake::MipArguments>(argc, argv);

    const auto problem = quake::ExtendedProblem::load_json(arguments.ProblemPath);
    const auto &weather_forecast = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast);
    const auto &weather_observed = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);

    quake::WorstCaseMipModel mip_model{&problem, arguments.IntervalStep, std::vector<quake::Forecast>{weather_forecast}};
    const auto solution_opt = mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    CHECK(solution_opt);

    quake::IndexEvaluator evaluator{problem};
    LOG(INFO) << "In-Sample Traffic Index: " << evaluator(*solution_opt, weather_forecast);
    LOG(INFO) << "Out-of-Sample Traffic Index: " << evaluator(*solution_opt, weather_observed);

    return EXIT_SUCCESS;
}