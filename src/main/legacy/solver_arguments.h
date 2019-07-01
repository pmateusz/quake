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

#ifndef QUAKE_SOLVER_ARGUMENTS_H
#define QUAKE_SOLVER_ARGUMENTS_H

#include <ostream>

#include <boost/config.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/date_time.hpp>

namespace quake {

    enum class ObjectiveType {
        Default,
        MinTotalAbsoluteError,
        MinMaxRelativeError,
        Envelope,
        SurplusEnvelope,
        Weighted
    };

    std::ostream &operator<<(std::ostream &out, ObjectiveType value);

    struct SolverArguments {
        static const std::string TOTAL_ABSOLUTE_ERROR_OBJECTIVE;
        static const std::string MIN_MAX_RELATIVE_ERROR_OBJECTIVE;
        static const std::string ENVELOPE_OBJECTIVE;
        static const std::string SURPLUS_ENVELOPE_OBJECTIVE;
        static const std::string WEIGHTED_OBJECTIVE;
        static const std::string DEFAULT_VALUE;

        static ObjectiveType parse(const std::string &value);

        static boost::optional<ObjectiveType> parseOrNone(const std::string &value);

        SolverArguments(boost::filesystem::path model_path, boost::filesystem::path output_path);

        boost::filesystem::path ModelPath;
        boost::filesystem::path OutputPath;
        boost::optional<boost::filesystem::path> CloudCoverPath;
        boost::optional<boost::filesystem::path> LoadInitialGuessPath;
        boost::optional<boost::filesystem::path> OutputInitialGuessPath;
        boost::optional<boost::filesystem::path> PreviousSolutionPath;
        boost::optional<int> MinScalingFactor;
        boost::optional<int> MaxScalingFactor;
        boost::optional<int> Repeats;
        boost::optional<int> FailureScalingFactor;
        boost::optional<boost::posix_time::time_duration> TimeLimit;
        boost::optional<int> MinJobs;
        boost::optional<int> MaxJobs;
        boost::optional<int> JobsStep;
        ObjectiveType ProblemObjectiveFunction;
        ObjectiveType SubproblemObjectiveFunction;
        bool PrintSolutions;
    };
}


#endif //QUAKE_SOLVER_ARGUMENTS_H
