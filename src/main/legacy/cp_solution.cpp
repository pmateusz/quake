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

#include "cp_solution.h"

#include <algorithm>
#include <numeric>
#include <vector>

quake::CpSolution::CpSolution()
        : CpSolution(std::vector<quake::CpSolution::Job>{}, std::vector<int64>{}) {}

quake::CpSolution::CpSolution(std::vector<quake::CpSolution::Job> actions)
        : CpSolution(actions, std::vector<int64>(actions.size(), 0)) {}

quake::CpSolution::CpSolution(std::vector<quake::CpSolution::Job> actions, std::vector<int64> final_buffer)
        : actions_(std::move(actions)),
          final_buffer_(std::move(final_buffer)) {
    std::size_t max_station_index = 0;
    for (const auto &action : actions_) {
        max_station_index = std::max(static_cast<std::size_t>(action.Station() + 1), max_station_index);
    }

    transferred_keys_ = std::vector<int64>(max_station_index, 0);
    for (const auto &action : actions_) {
        transferred_keys_[action.Station()] += action.KeysTransferred();
    }

    total_key_rate_ = std::accumulate(std::cbegin(transferred_keys_),
                                      std::cend(transferred_keys_),
                                      static_cast<int64>(0));

    if (transferred_keys_.empty()) {
        min_station_key_rate_ = 0;
        max_station_key_rate_ = 0;
    } else {
        const auto min_max_it = std::minmax_element(std::cbegin(transferred_keys_),
                                                    std::cend(transferred_keys_));
        min_station_key_rate_ = *min_max_it.first;
        max_station_key_rate_ = *min_max_it.second;
    }
}

quake::CpSolution quake::CpSolution::Load(const boost::filesystem::path &file_path) {
    std::ifstream input_stream;

    VLOG(1) << "Loading solution from " << file_path;
    input_stream.open(file_path.string(), std::ifstream::in);
    if (!input_stream.is_open()) {
        LOG(FATAL) << "Failed to open " << file_path;
    }

    nlohmann::json json_object;
    input_stream >> json_object;

    const auto solutions = json_object["solutions"].get<std::vector<quake::CpSolution> >();

    return json_object.get<quake::CpSolution>();
}


quake::CpSolution::Job::Job(int64 start, int64 duration, int64 station, int64 key_rate)
        : start_{start},
          duration_{duration},
          station_{station},
          keys_transferred_{key_rate} {}

void quake::from_json(const nlohmann::json &json, quake::CpSolution::Job &value) {
    value.start_ = json["start"].get<int64>();
    value.duration_ = json["duration"].get<int64>();
    value.station_ = json["station"].get<int64>();
    value.keys_transferred_ = json["key_rate"].get<int64>();
}

void quake::to_json(nlohmann::json &json, const quake::CpSolution &value) {
    json["total_key_rate"] = value.total_key_rate_;
    json["max_station_key_rate"] = value.max_station_key_rate_;
    json["min_station_key_rate"] = value.min_station_key_rate_;
    json["station_key_rate_cumulative"] = value.transferred_keys_;
    json["final_buffer"] = value.final_buffer_;
    json["jobs"] = value.actions_;
}

void quake::from_json(const nlohmann::json &json, quake::CpSolution &value) {
    value.total_key_rate_ = json["total_key_rate"].get<int64>();
    value.max_station_key_rate_ = json["max_station_key_rate"].get<int64>();
    value.min_station_key_rate_ = json["min_station_key_rate"].get<int64>();
    value.transferred_keys_ = json["station_key_rate_cumulative"].get<std::vector<int64> >();

    if (json.find("final_buffer") != json.end()) {
        value.final_buffer_ = json["final_buffer"].get<std::vector<int64> >();
    } else {
        value.final_buffer_ = std::vector<int64>(value.transferred_keys_.size(), 0);
    }

    value.actions_ = json["jobs"].get<std::vector<CpSolution::Job> >();
}

void quake::to_json(nlohmann::json &json, const quake::CpSolution::Job &value) {
    json["start"] = value.start_;
    json["duration"] = value.duration_;
    json["station"] = value.station_;
    json["key_rate"] = value.keys_transferred_;
}

std::ostream &operator<<(std::ostream &out, const quake::CpSolution::Job &job) {
    out << typeid(job).name()
        << " " << job.Station()
        << " [" << job.Start() << ":" << job.Start() + job.Duration() << "] "
        << job.KeysTransferred();

    return out;
}
