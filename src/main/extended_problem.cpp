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

#include <unordered_set>

#include <glog/logging.h>

#include "extended_problem.h"

#include "util/math.h"
#include "util/json.h"
#include "forecast.h"

quake::ExtendedProblem::ExtendedProblem()
        : ExtendedProblem(MetaData{}, std::vector<StationData>{}) {}

quake::ExtendedProblem::ExtendedProblem(ExtendedProblem::MetaData metadata,
                                        std::vector<ExtendedProblem::StationData> station_data)
        : ExtendedProblem(metadata, station_data, std::unordered_map<std::string, Forecast>{}) {}

quake::ExtendedProblem::ExtendedProblem(ExtendedProblem::MetaData metadata,
                                        std::vector<ExtendedProblem::StationData> station_data,
                                        std::unordered_map<std::string, Forecast> forecasts)
        : metadata_{std::move(metadata)},
          station_data_{std::move(station_data)},
          forecasts_{std::move(forecasts)} {

    std::unordered_set<GroundStation> ground_stations;
    for (const auto &local_station_data : station_data_) {
        ground_stations.emplace(local_station_data.Station);
    }

    ground_stations_.reserve(station_data.size());
    std::copy(std::cbegin(ground_stations), std::cend(ground_stations), std::back_inserter(ground_stations_));
    std::sort(std::begin(ground_stations_), std::end(ground_stations_), [](const GroundStation &left, const GroundStation &right) -> bool {
        return left.name() < right.name();
    });
}

quake::ExtendedProblem quake::ExtendedProblem::Round(unsigned int decimal_places) const {
    std::vector<StationData> rounded_station_data;

    for (const auto &station_data: station_data_) {
        std::vector<CommunicationWindowData> communication_window_data;
        communication_window_data.reserve(station_data.CommunicationWindows.size());
        for (const auto &window_data : station_data.CommunicationWindows) {
            std::vector<double> rounded_elevation_angle;
            rounded_elevation_angle.reserve(window_data.ElevationAngle.size());
            for (auto elevation_angle: window_data.ElevationAngle) {
                rounded_elevation_angle.emplace_back(util::round(elevation_angle, decimal_places));
            }

            std::vector<double> rounded_key_rate;
            rounded_key_rate.reserve(window_data.KeyRate.size());
            for (auto key_rate : window_data.KeyRate) {
                rounded_key_rate.emplace_back(util::round(key_rate, decimal_places));
            }

            communication_window_data.emplace_back(window_data.Period, std::move(rounded_elevation_angle), std::move(rounded_key_rate));
        }

        rounded_station_data.emplace_back(station_data.Station,
                                          station_data.TransferShare,
                                          station_data.InitialBuffer,
                                          station_data.KeyConsumption,
                                          std::move(communication_window_data));
    }

    return {metadata_, std::move(rounded_station_data), forecasts_};
}

std::vector<boost::posix_time::time_period> quake::ExtendedProblem::TransferWindows(const GroundStation &station) const {
    const auto &station_data = GetStationData(station);
    std::vector<boost::posix_time::time_period> communication_periods;
    communication_periods.reserve(station_data.CommunicationWindows.size());
    for (const auto &window_data : station_data.CommunicationWindows) {
        communication_periods.emplace_back(window_data.Period);
    }
    return communication_periods;
}

double zero(boost::posix_time::ptime) { return 0.0; }

double quake::ExtendedProblem::KeyRate(const quake::GroundStation &station,
                                       const boost::posix_time::ptime &datetime,
                                       ExtendedProblem::WeatherSample sample) const {
    return KeyRate(station, boost::posix_time::time_period{datetime, boost::posix_time::seconds(1)}, sample);
}

double quake::ExtendedProblem::KeyRate(const quake::GroundStation &station,
                                       const boost::posix_time::time_period &period,
                                       ExtendedProblem::WeatherSample sample) const {
    switch (sample) {
        case ExtendedProblem::WeatherSample::None:
            return KeyRate(station, period, zero);
        case ExtendedProblem::WeatherSample::Forecast:
            return KeyRate(station, period, forecasts_.at("forecast"));
        case ExtendedProblem::WeatherSample::Real:
            return KeyRate(station, period, forecasts_.at("real"));
        default:
            LOG(FATAL) << "Weather sample " << static_cast<int>(sample) << " is not supported";
    }
}

double quake::ExtendedProblem::KeyRate(const quake::GroundStation &station,
                                       const boost::posix_time::time_period &period,
                                       const quake::Forecast &forecast) const {
    const auto callback = [&station, &forecast](boost::posix_time::ptime time_point) -> double {
        const auto cloud_cover = forecast.GetCloudCover(station, time_point);
        const auto normalized_cloud_cover = static_cast<double>(cloud_cover) / 100.0;
        return normalized_cloud_cover;
    };

    return KeyRate(station, period, callback);
}

double quake::ExtendedProblem::KeyRate(const quake::GroundStation &station,
                                       const boost::posix_time::time_period &period,
                                       const std::function<double(boost::posix_time::ptime)> &weather_callback) const {
    double total_key_rate = 0.0;
    const auto &station_data = GetStationData(station);
    for (const auto &window : station_data.CommunicationWindows) {
        if (window.Period.is_after(period.end())) {
            break;
        }

        if (window.Period.intersects(period)) {
            const auto intersection = window.Period.intersection(period);
            for (auto time_point = intersection.begin(); time_point < intersection.end(); time_point += boost::posix_time::seconds(1)) {
                const auto cloud_cover = weather_callback(time_point);
                CHECK_GE(cloud_cover, 0.0);
                CHECK_LE(cloud_cover, 1.0);

                const auto index = (time_point - window.Period.begin()).total_seconds();
                const auto key_rate = (1.0 - cloud_cover) * window.KeyRate.at(index);
                CHECK_GE(key_rate, 0.0);

                total_key_rate += key_rate;
            }
        }
    }
    return total_key_rate;
}

double quake::ExtendedProblem::ElevationAngle(const quake::GroundStation &station, const boost::posix_time::ptime &datetime) const {
    const auto &station_data = GetStationData(station);
    for (const auto &window : station_data.CommunicationWindows) {
        if (window.Period.is_after(datetime)) {
            return 0;
        }

        if (window.Period.contains(datetime)) {
            const auto index = (datetime - window.Period.begin()).total_seconds();
            return window.ElevationAngle.at(index);
        }
    }
    return 0;
}

const quake::ExtendedProblem::StationData &quake::ExtendedProblem::GetStationData(const quake::GroundStation &station) const {
    for (const auto &station_data: station_data_) {
        if (station_data.Station == station) {
            return station_data;
        }
    }

    LOG(FATAL) << "GroundStation not found: " << station;
}

void quake::to_json(nlohmann::json &json, const quake::ExtendedProblem &problem) {
    nlohmann::json object;

    object["metadata"] = problem.metadata_;
    object["stations"] = problem.station_data_;

    if (!problem.forecasts_.empty()) {
        object["forecasts"] = problem.forecasts_;
    }

    json = object;
}

quake::ExtendedProblem quake::ExtendedProblem::load_json(const boost::filesystem::path &file_path) {
    std::ifstream input_stream;
    input_stream.open(file_path.string(), std::ifstream::in);

    nlohmann::json json_object;
    input_stream >> json_object;
    input_stream.close();

    ExtendedProblem problem = json_object;
    return problem;
}

void quake::to_json(nlohmann::json &json, const quake::ExtendedProblem::MetaData &metadata) {
    nlohmann::json object;

    object["observation_period"] = metadata.Period;
    object["switch_duration"] = metadata.SwitchDuration;

    json = object;
}

void quake::to_json(nlohmann::json &json, const quake::ExtendedProblem::StationData &station_data) {
    nlohmann::json object;

    object["station"] = station_data.Station;
    object["transfer_share"] = station_data.TransferShare;
    object["initial_buffer"] = station_data.InitialBuffer;
    object["key_consumption"] = station_data.KeyConsumption;
    object["communication_windows"] = station_data.CommunicationWindows;

    json = object;
}

void quake::to_json(nlohmann::json &json, const quake::ExtendedProblem::CommunicationWindowData &communication_window_data) {
    nlohmann::json object;

    object["period"] = communication_window_data.Period;
    object["elevation"] = communication_window_data.ElevationAngle;
    object["key_rate"] = communication_window_data.KeyRate;

    json = object;
}

void quake::from_json(const nlohmann::json &json, ExtendedProblem::MetaData &metadata) {
    const auto switch_duration = json.at("switch_duration").get<boost::posix_time::time_duration>();
    const auto observation_period = util::from_json<boost::posix_time::time_period>(json.at("observation_period"));

    ExtendedProblem::MetaData metadata_object{observation_period, switch_duration};
    metadata = metadata_object;
}

void quake::from_json(const nlohmann::json &json, ExtendedProblem::StationData &station_data) {
    const auto station = json.at("station").get<quake::GroundStation>();
    const auto transfer_share = json.at("transfer_share").get<double>();
    const auto initial_buffer = json.at("initial_buffer").get<int>();
    const auto key_consumption = json.at("key_consumption").get<int>();
    const auto communication_windows = json.at("communication_windows").get<std::vector<ExtendedProblem::CommunicationWindowData>>();

    ExtendedProblem::StationData station_data_object{station, transfer_share, initial_buffer, key_consumption, communication_windows};
    station_data = station_data_object;
}

void quake::from_json(const nlohmann::json &json, ExtendedProblem::CommunicationWindowData &communication_window_data) {
    auto elevation = json.at("elevation").get<std::vector<double>>();
    auto key_rate = json.at("key_rate").get<std::vector<double>>();
    auto time_period = util::from_json<boost::posix_time::time_period>(json.at("period"));

    ExtendedProblem::CommunicationWindowData communication_window_data_object{time_period, std::move(elevation), std::move(key_rate)};
    communication_window_data = communication_window_data_object;
}

void quake::from_json(const nlohmann::json &json, quake::ExtendedProblem &problem) {
    const auto metadata = json.at("metadata").get<ExtendedProblem::MetaData>();
    const auto stations = json.at("stations").get<std::vector<ExtendedProblem::StationData>>();

    std::unordered_map<std::string, Forecast> forecasts;
    const auto find_it = json.find("forecasts");
    if (find_it != std::end(json)) {
        forecasts = find_it->get<std::unordered_map<std::string, Forecast>>();
    }

    ExtendedProblem problem_object{metadata, stations, std::move(forecasts)};
    problem = problem_object;
}

