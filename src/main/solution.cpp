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

#include "solution.h"
#include "solution_json_reader.h"

quake::Solution::Solution(std::unordered_map<quake::GroundStation, int64> final_buffers)
        : Solution({}, std::move(final_buffers)) {}

quake::Solution::Solution(
        std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> > observations,
        std::unordered_map<quake::GroundStation, int64> final_buffers)
        : observations_{std::move(observations)},
          final_buffers_{std::move(final_buffers)} {
    CHECK_EQ(observations_.size(), final_buffers_.size());
}

quake::Solution quake::Solution::Load(const boost::filesystem::path &path) {
    std::ifstream input_stream;
    input_stream.open(path.string(), std::ifstream::in);
    LOG_IF(FATAL, !input_stream.is_open()) << "Failed to open " << path;

    // TODO: replace by json reader
    SolutionJsonReader<std::ifstream> solution_reader{std::move(input_stream)};
    return solution_reader.Read();
}

const std::vector<boost::posix_time::time_period> &quake::Solution::ObservationWindows(const quake::GroundStation &station) const {
    const auto find_it = observations_.find(station);
    if (find_it != std::cend(observations_)) {
        return find_it->second;
    }
    return NO_OBSERVATIONS;
}

const std::vector<boost::posix_time::time_period> quake::Solution::NO_OBSERVATIONS = std::vector<boost::posix_time::time_period>();

std::vector<quake::GroundStation> quake::Solution::Stations() const {
    std::vector<quake::GroundStation> stations;
    stations.reserve(final_buffers_.size());

    for (const auto &element : final_buffers_) {
        stations.emplace_back(element.first);
    }

    return stations;
}

int64 quake::Solution::FinalBuffer(const quake::GroundStation &station) const {
    return final_buffers_.at(station);
}
