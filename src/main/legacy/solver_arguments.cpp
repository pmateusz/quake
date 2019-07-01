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

#include "solver_arguments.h"

#include <glog/logging.h>

const std::string quake::SolverArguments::TOTAL_ABSOLUTE_ERROR_OBJECTIVE = {"total-absolute-error"};
const std::string quake::SolverArguments::MIN_MAX_RELATIVE_ERROR_OBJECTIVE = {"min-max-relative-error"};
const std::string quake::SolverArguments::ENVELOPE_OBJECTIVE = {"envelope"};
const std::string quake::SolverArguments::DEFAULT_VALUE = {"default"};
const std::string quake::SolverArguments::SURPLUS_ENVELOPE_OBJECTIVE = {"surplus-envelope"};
const std::string quake::SolverArguments::WEIGHTED_OBJECTIVE = {"weighted-objective"};

std::ostream &quake::operator<<(std::ostream &out, quake::ObjectiveType value) {
    switch (value) {
        case ObjectiveType::Default:
            out << quake::SolverArguments::DEFAULT_VALUE;
            break;
        case ObjectiveType::MinTotalAbsoluteError:
            out << quake::SolverArguments::TOTAL_ABSOLUTE_ERROR_OBJECTIVE;
            break;
        case ObjectiveType::MinMaxRelativeError:
            out << quake::SolverArguments::MIN_MAX_RELATIVE_ERROR_OBJECTIVE;
            break;
        case ObjectiveType::Envelope:
            out << quake::SolverArguments::ENVELOPE_OBJECTIVE;
            break;
        case ObjectiveType::SurplusEnvelope:
            out << quake::SolverArguments::SURPLUS_ENVELOPE_OBJECTIVE;
            break;
        case ObjectiveType::Weighted:
            out << quake::SolverArguments::WEIGHTED_OBJECTIVE;
            break;
    }

    return out;
}


quake::ObjectiveType quake::SolverArguments::parse(const std::string &value) {
    const auto opt_value = parseOrNone(value);

    if (opt_value) {
        return opt_value.get();
    }

    LOG(FATAL) << "Unknown value of objective-function: " << value;
    return quake::ObjectiveType::Default;
}

boost::optional<quake::ObjectiveType> quake::SolverArguments::parseOrNone(const std::string &value) {
    if (value.empty()) {
        return quake::ObjectiveType::Default;
    }

    std::string value_to_use{value};
    boost::to_lower(value_to_use);
    if (value_to_use == DEFAULT_VALUE || value_to_use == TOTAL_ABSOLUTE_ERROR_OBJECTIVE) {
        return quake::ObjectiveType::MinTotalAbsoluteError;
    }

    if (value_to_use == MIN_MAX_RELATIVE_ERROR_OBJECTIVE) {
        return quake::ObjectiveType::MinMaxRelativeError;
    }

    if (value_to_use == ENVELOPE_OBJECTIVE) {
        return quake::ObjectiveType::Envelope;
    }

    if (value_to_use == SURPLUS_ENVELOPE_OBJECTIVE) {
        return quake::ObjectiveType::SurplusEnvelope;
    }

    if (value_to_use == WEIGHTED_OBJECTIVE) {
        return quake::ObjectiveType::Weighted;
    }

    return boost::none;
}


quake::SolverArguments::SolverArguments(boost::filesystem::path model_path,
                                        boost::filesystem::path output_path)
        : ModelPath{std::move(model_path)},
          OutputPath{std::move(output_path)},
          CloudCoverPath{boost::none},
          LoadInitialGuessPath{boost::none},
          OutputInitialGuessPath{boost::none},
          ProblemObjectiveFunction{ObjectiveType::Default},
          SubproblemObjectiveFunction{ObjectiveType::Default} {}