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

#include <fstream>
#include <iostream>
#include <unordered_map>

#include <boost/config.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <glog/logging.h>

#include "forecast.h"
#include "util/json.h"

quake::Forecast::Forecast()
        : Forecast(std::unordered_map<GroundStation, Series>{}) {}

quake::Forecast::Forecast(std::unordered_map<GroundStation, Series> index)
        : index_{std::move(index)} {}

quake::Forecast::SeriesValueType quake::Forecast::GetCloudCover(const GroundStation &station, const boost::posix_time::ptime &time) const {
    if (station == GroundStation::None) {
        return 0;
    }

    const auto find_it = index_.find(station);
    CHECK(find_it != std::cend(index_));
    return find_it->second(time);
}

quake::Forecast quake::Forecast::load_csv(const boost::filesystem::path &input_file) {
    std::ifstream input_stream;
    input_stream.open(input_file.string(), std::ifstream::in);
    if (!input_stream.is_open()) {
        LOG(FATAL) << "Failed to open file: " << input_file;
    }

    std::string line;
    if (!std::getline(input_stream, line)) {
        LOG(FATAL) << "Failed to read headers";
    }

    std::unordered_map<GroundStation, std::vector<std::pair<boost::posix_time::ptime, SeriesValueType> > > raw_index;
    boost::tokenizer<boost::escaped_list_separator<char> > tokenizer{line};
    while (std::getline(input_stream, line)) {
        boost::trim_right(line);
        tokenizer.assign(line);
        std::vector<std::string> tokens{tokenizer.begin(), tokenizer.end()};

        if (tokens.size() != 3) {
            LOG(FATAL) << "Invalid format: " << line;
        }

        const auto local_time = boost::posix_time::time_from_string(tokens[0]);
        const auto local_ground_station = GroundStation::FromName(tokens[1]);
        const auto local_cloud_cover = std::stoi(tokens[2]);
        auto find_it = raw_index.find(local_ground_station);
        if (find_it == std::end(raw_index)) {
            raw_index.emplace(local_ground_station,
                              std::vector<std::pair<boost::posix_time::ptime, SeriesValueType> >{
                                      std::make_pair(local_time, local_cloud_cover)});
        } else {
            find_it->second.emplace_back(local_time, local_cloud_cover);
        }
    }
    input_stream.close();

    const auto sort_by_first_element = [](const std::pair<boost::posix_time::ptime, SeriesValueType> &left,
                                          const std::pair<boost::posix_time::ptime, SeriesValueType> &right)
            -> bool { return left.first <= right.first; };

    for (auto &first_level_entry : raw_index) {
        std::sort(std::begin(first_level_entry.second), std::end(first_level_entry.second), sort_by_first_element);
    }

    std::unordered_map<GroundStation, Series> index;

    for (const auto &raw_entry : raw_index) {
        const auto start_time = raw_entry.second.front().first;
        const auto end_time = raw_entry.second.back().first;
        boost::posix_time::time_period series_period{start_time, end_time};

        std::vector<boost::posix_time::time_duration> time_difference;
        for (auto position = 1; position < raw_entry.second.size(); ++position) {
            const auto local_difference = raw_entry.second[position].first - raw_entry.second[position - 1].first;
            if (local_difference != boost::posix_time::seconds(0)) {
                time_difference.push_back(local_difference);
            }
        }

        for (auto position = 1; position < time_difference.size(); ++position) {
            LOG_IF(WARNING, time_difference[position] != time_difference[position - 1])
                            << "Frequency of updates is not constant for station " << raw_entry.first
                            << ", example  " << time_difference[position - 1] << " vs " << time_difference[position];
        }

        CHECK(!time_difference.empty());
        const auto update_frequency = time_difference[0];

        std::vector<SeriesValueType> values(time_difference.size() + 1, 0);
        for (const auto &observation : raw_entry.second) {
            const auto time_offset = observation.first - series_period.begin();
            CHECK(!time_offset.is_negative());

            const auto position = time_offset.total_seconds() / update_frequency.total_seconds();
            CHECK_LT(position, values.size());
            values[position] = observation.second;
        }

        index.emplace(raw_entry.first, std::move(Series{update_frequency, series_period, values}));
    }

    return Forecast{std::move(index)};
}

void quake::from_json(const nlohmann::json &json, Forecast &forecast) {
    auto index = json.at("index").get<std::vector<boost::posix_time::ptime>>();

    // check index is ascending with a constant step
    CHECK(!index.empty());
    boost::posix_time::time_duration step;
    if (index.size() >= 2) {
        auto prev_value = index.front();

        step = index[1] - index[0];
        for (auto index_pos = 1; index_pos < index.size(); ++index_pos) {
            auto current_value = index[index_pos];
            CHECK_LT(prev_value, current_value);

            auto current_length = current_value - prev_value;
            CHECK_EQ(current_length, step);
            prev_value = current_value;
        }
    }

    boost::posix_time::time_period period{index.front(), index.back() + step};
    CHECK_EQ(period.length().total_seconds(), index.size() * step.total_seconds());

    std::unordered_map<GroundStation, Forecast::Series> series;
    for (const auto &json_element : json.at("stations")) {
        auto station = json_element.at("station").get<GroundStation>();
        auto cloud_cover = json_element.at("cloud_cover").get<std::vector<Forecast::SeriesValueType>>();

        CHECK_EQ(index.size(), cloud_cover.size());
        series.emplace(station, Forecast::Series(step, period, cloud_cover));
    }

    Forecast forecast_object{std::move(series)};
    forecast = forecast_object;
}

void quake::to_json(nlohmann::json &json, const quake::Forecast &forecast) {
    nlohmann::json json_object;
    std::vector<boost::posix_time::ptime> index;
    std::unordered_map<std::string, nlohmann::json> stations;

    if (!forecast.index_.empty()) {
        const auto &series = std::begin(forecast.index_)->second;
        for (auto index_value = series.Period().begin(); index_value <= series.Period().end(); index_value += series.UpdateFrequency()) {
            index.emplace_back(index_value);
        }

        for (const auto &element : forecast.index_) {
            nlohmann::json element_object;
            element_object["station"] = element.first;
            element_object["cloud_cover"] = element.second.Values();
            stations.emplace(element.first.name(), element_object);
        }
    }

    json_object["index"] = index;
    json_object["stations"] = stations;
    json = json_object;
}
