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

#include "util/json.h"

#include "solution.h"

quake::Solution::Solution()
        : Solution({}, {}, {}) {}

quake::Solution::Solution(std::unordered_map<GroundStation, int64> final_buffers)
        : Solution({}, {}, std::move(final_buffers)) {}

quake::Solution::Solution(Metadata metadata,
                          std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period> > observations,
                          std::unordered_map<GroundStation, int64> final_buffers)
        : metadata_{std::move(metadata)},
          observations_{std::move(observations)},
          final_buffers_{std::move(final_buffers)} {
    CHECK_LE(observations_.size(), final_buffers_.size());

    stations_.reserve(final_buffers_.size());
    for (const auto &element : final_buffers_) {
        stations_.emplace_back(element.first);
    }
}

quake::Solution quake::Solution::load_json(const boost::filesystem::path &path) {
    std::ifstream input_stream;
    input_stream.open(path.string(), std::ifstream::in);
    LOG_IF(FATAL, !input_stream.is_open()) << "Failed to open " << path;

    nlohmann::json object;
    input_stream >> object;
    input_stream.close();

    return object.get<Solution>();
}

const std::vector<boost::posix_time::time_period> &quake::Solution::ObservationWindows(const GroundStation &station) const {
    const auto find_it = observations_.find(station);
    if (find_it != std::cend(observations_)) {
        return find_it->second;
    }
    return NO_OBSERVATIONS;
}

const std::vector<boost::posix_time::time_period> quake::Solution::NO_OBSERVATIONS = std::vector<boost::posix_time::time_period>();

int64 quake::Solution::FinalBuffer(const GroundStation &station) const {
    return final_buffers_.at(station);
}

void quake::to_json(nlohmann::json &json, const Solution &solution) {
    nlohmann::json object;
    object["observations"] = solution.observations_;
    object["final_buffers"] = solution.final_buffers_;
    object["metadata"] = solution.metadata_;
    json = std::move(object);
}

void quake::from_json(const nlohmann::json &json, Solution &solution) {
    auto metadata = json.at("metadata").get<Metadata>();

    auto json_observations = json.at("observations").get<std::unordered_map<GroundStation, std::vector<nlohmann::json> >>();

    std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period> > observations;
    for (const auto &element : json_observations) {
        std::vector<boost::posix_time::time_period> periods;
        periods.reserve(element.second.size());

        for (const auto &json_period : element.second) {
            periods.emplace_back(util::from_json<boost::posix_time::time_period>(json_period));
        }

        observations.emplace(element.first, std::move(periods));
    }

    auto final_buffers = json.at("final_buffers").get<std::unordered_map<GroundStation, int64>>();

    Solution local_solution(std::move(metadata), std::move(observations), std::move(final_buffers));
    solution = std::move(local_solution);
}
