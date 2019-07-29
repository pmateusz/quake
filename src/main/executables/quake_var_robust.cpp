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


#include "extended_problem.h"
#include "index/robust_var_index_mip_model.h"
#include "index/index_evaluator.h"
#include "executables/mip_arguments.h"


int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<quake::ScenarioIndexMipArguments>(argc, argv);
    const auto problem = quake::ExtendedProblem::load_json(arguments.ProblemPath);

    quake::RobustVarIndexMipModel robust_mip_model{&problem, arguments.IntervalStep, arguments.TargetTrafficIndex};
    auto solution_opt = robust_mip_model.Solve(arguments.TimeLimit, arguments.GapLimit, boost::none);
    if (solution_opt) {
        solution_opt->GetMetadata().SetProperty(quake::Metadata::Property::SolutionType, quake::Metadata::SolutionType::Test);

        quake::IndexEvaluator evaluator{problem};
        LOG(INFO) << "In Sample: " << evaluator(*solution_opt, problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast));
        LOG(INFO) << "Out of Sample: " << evaluator(*solution_opt, problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real));

        // quake::util::Save(*solution_opt, arguments.SolutionFile);
    } else {
        LOG(FATAL) << "Failed to solve the problem";
    }

    return EXIT_SUCCESS;
}