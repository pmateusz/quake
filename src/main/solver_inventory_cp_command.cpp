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

#include "solver_inventory_cp_command.h"

#include <fstream>

#include <boost/config.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include <glog/logging.h>

#include "util/error.h"

#include "legacy/independent_stage_solver.h"
#include "legacy/max_envelope_solver.h"
#include "legacy/min_envelope_solver.h"
#include "legacy/preserve_envelope_max_key_solver.h"
#include "legacy/weighted_sum_solver.h"
#include "legacy/cp_solution.h"
#include "legacy/cp_solution_json_writer.h"

#include "solution_json_reader.h"
#include "forecast.h"
#include "inferred_model.h"


quake::SolverInventoryCpCommand::SolverInventoryCpCommand(quake::SolverArguments args)
        : args_{std::move(args)} {}

void quake::SolverInventoryCpCommand::Run() {
    auto model = InferredModel::Load(args_.ModelPath);

    if (args_.CloudCoverPath) {
        // if could cover is enabled the final number of keys need to be divided by 100
        const auto forecast = Forecast::load_csv(args_.CloudCoverPath->string());
        model.Apply(forecast);
    }

    if (args_.PreviousSolutionPath) {
        const auto solution = Solution::Load(*args_.PreviousSolutionPath);
        model.Apply(solution);
    }

    const auto initial_guess = ComputeOrLoadInitialGuess(model);
    auto min_transfer_size = initial_guess.MinTransferSize();
    if (min_transfer_size < 100) {
        min_transfer_size = 0;
    } else {
        min_transfer_size = 100;
    }

    boost::optional<CpSolution> second_stage_solution_opt = boost::none;
    if (args_.ProblemObjectiveFunction == ObjectiveType::Weighted) {
        WeightedSumSolver solver{&model, min_transfer_size, &args_};
        second_stage_solution_opt = solver.Solve(initial_guess);
    } else {
        MinEnvelopeSolver solver{&model, min_transfer_size, &args_};
        const auto first_stage_solution_opt = solver.Solve(initial_guess);
        CHECK(first_stage_solution_opt);

        auto current_iteration = 0;
        static const auto MAX_ITERATIONS = 5;
        while (!second_stage_solution_opt && current_iteration < MAX_ITERATIONS) {
            LOG_IF(WARNING, current_iteration > 0) << "Failed to find a feasible solution, retry: "
                                                   << current_iteration;

            PreserveEnvelopeMaxKeySolver second_stage_solver{&model, min_transfer_size, &args_};
            second_stage_solution_opt = second_stage_solver.Solve(first_stage_solution_opt.get());
            ++current_iteration;
        }

        LOG_IF(WARNING, current_iteration > 0) << "Failed to find a feasible solution in the second stage."
                                                  " Keeping the first stage solution";
        second_stage_solution_opt = first_stage_solution_opt;
    }

    CHECK(second_stage_solution_opt) << "Infeasible solution";
    // 1 stage: MaxEnvelope                             London 310392; Glasgow 148758; Thurso 12356; Manchester 148774; Birmingham 186151; Bristol 124069; Ipswich 186027; Cambridge 61962; York 62082
    // 1 stage: MinEnvelope                             London 467512; Glasgow 225478; Thurso 110504; Manchester 224754; Birmingham 280887; Bristol 205737; Ipswich 280863; Cambridge 103054; York 147518
    // 2 stages: MinEnvelope + PreserveEnvelopeMaxKey - London 468500; Glasgow 224786; Thurso 110504; Manchester 224990; Birmingham 280887; Bristol 205737; Ipswich 280863; Cambridge 103054; York 147518
    // 2 stages: MaxEnvelope + PreserveEnvelopeMaxKey - London 310392; Glasgow 148758; Thurso 12356; Manchester 148774; Birmingham 186151; Bristol 124069; Ipswich 186027; Cambridge 61962; York 62082

    if (args_.CloudCoverPath) {
        // if could cover is enabled the final number of keys need to be divided by 100
        std::vector<CpSolution::Job> rescaled_jobs;
        for (auto &job : second_stage_solution_opt->Jobs()) {
            rescaled_jobs.emplace_back(job.Start(), job.Duration(), job.Station(), job.KeysTransferred() / 100);
        }

        std::vector<int64> final_keys(model.StationCount(), 0);
        for (auto station_index = 1; station_index < model.StationCount(); ++station_index) {
            final_keys[station_index] = second_stage_solution_opt->FinalBuffer(station_index) / 100;
        }

        CpSolution patched_solution{std::move(rescaled_jobs), std::move(final_keys)};
        Save(model, patched_solution);
    } else {
        Save(model, second_stage_solution_opt.get());
    }
}

quake::CpSolution quake::SolverInventoryCpCommand::ComputeOrLoadInitialGuess(const quake::InferredModel &model) const {

    if (args_.LoadInitialGuessPath) {
        LOG(WARNING) << "Loading a precomputed solution from file: " << args_.LoadInitialGuessPath;

        std::ifstream file_stream;
        file_stream.open(args_.LoadInitialGuessPath->string(), std::ifstream::in);
        if (!file_stream.is_open()) {
            LOG(FATAL) << "Failed to open the solution file: " << args_.LoadInitialGuessPath;
        }
        const CpSolution initial_guess = nlohmann::json::parse(file_stream);
        file_stream.close();

        model.CheckConsistency(initial_guess);

        return initial_guess;
    } else {
        IndependentStageSolver solver{&model};
        const auto initial_guess = solver.Solve(args_);

        if (args_.OutputInitialGuessPath) {
            std::ofstream file_stream;
            file_stream.open(args_.OutputInitialGuessPath->string(), std::ofstream::out | std::ofstream::trunc);
            if (!file_stream.is_open()) {
                LOG(FATAL) << "Failed to open the solution file " << args_.OutputInitialGuessPath;
            }

            nlohmann::json json_body = initial_guess;
            file_stream << std::setw(4) << json_body;
            file_stream.close();
        }

        return initial_guess;
    }
}

void quake::SolverInventoryCpCommand::Save(const InferredModel &model, const quake::CpSolution &solution) const {
    std::ofstream output_file;
    output_file.open(args_.OutputPath.string(), std::fstream::out | std::fstream::trunc);
    if (!output_file.is_open()) {
        throw util::OnFailedSaveOutput(args_.OutputPath);
    }

    CpSolutionJsonWriter<std::ofstream> json_writer{std::move(output_file)};
    json_writer.Write(model, solution);
    json_writer.Close();
}
