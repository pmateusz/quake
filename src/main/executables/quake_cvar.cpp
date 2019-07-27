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

#include <glog/logging.h>

#include <boost/optional.hpp>

#include "index/worst_case_mip_model.h"
#include "index/cvar_mip_model.h"
#include "mip_arguments.h"


int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<quake::ScenarioIndexMipArguments>(argc, argv);
    const auto problem = quake::ExtendedProblem::load_json(arguments.ProblemPath);
    const auto forecast_scenarios = problem.GetWeatherSamples(quake::ExtendedProblem::WeatherSample::Scenario, arguments.NumScenarios);

    quake::CVarMipModel mip_model(&problem, arguments.IntervalStep, forecast_scenarios, arguments.TargetTrafficIndex, 0.05);
    auto solution_opt = mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    if (solution_opt) {
        solution_opt->GetMetadata().SetProperty(quake::Metadata::Property::SolutionType, quake::Metadata::SolutionType::Reference);

        quake::util::Save(*solution_opt, arguments.SolutionFile);
    } else {
        LOG(FATAL) << "Failed to solve the problem";
    }

    return EXIT_SUCCESS;
}