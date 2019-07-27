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
#include <boost/format.hpp>

#include "extended_problem.h"
#include "index/worst_case_mip_model.h"
#include "mip_arguments.h"
#include "metadata.h"

DEFINE_string(solution_prefix, "solution", "Output prefix appended to the solution file.");

struct Arguments : public quake::MipArguments {

    Arguments()
            : MipArguments() {}

    void Fill() override {
        MipArguments::Fill();

        CHECK(!FLAGS_solution_prefix.empty());
        SolutionPrefix = FLAGS_solution_prefix;
    }

    std::string SolutionPrefix;
};


void Solve(const Arguments &args,
           const quake::ExtendedProblem &problem,
           const quake::Forecast &forecast,
           quake::Metadata::SolutionType solution_type,
           const boost::filesystem::path &solution_file_path) {
    quake::WorstCaseMipModel mip_model{&problem, args.IntervalStep, {forecast}};
    auto solution_opt = mip_model.Solve(args.TimeLimit, args.GapLimit, boost::none);

    if (solution_opt) {
        solution_opt->GetMetadata().SetProperty(quake::Metadata::Property::SolutionType, solution_type);

        quake::util::Save(*solution_opt, solution_file_path);
    } else {
        LOG(FATAL) << "Failed to solve the problem";
    }
}

int main(int argc, char *argv[]) {
    const auto arguments = quake::SetupLogsAndParseArgs<Arguments>(argc, argv);
    const auto problem = quake::ExtendedProblem::load_json(arguments.ProblemPath);

    {
        LOG(INFO) << "Solving model with weather forecast";
        const auto &forecast = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Forecast);
        boost::filesystem::path solution_file_path = (boost::format("%1%_deterministic_forecast.json") % arguments.SolutionPrefix).str();
        Solve(arguments, problem, forecast, quake::Metadata::SolutionType::Test, solution_file_path);
    }

    {
        LOG(INFO) << "Solving model with weather forecast";
        const auto &real_weather = problem.GetWeatherSample(quake::ExtendedProblem::WeatherSample::Real);
        boost::filesystem::path solution_file_path = (boost::format("%1%_deterministic_real.json") % arguments.SolutionPrefix).str();
        Solve(arguments, problem, real_weather, quake::Metadata::SolutionType::Reference, solution_file_path);
    }

    return EXIT_SUCCESS;
}