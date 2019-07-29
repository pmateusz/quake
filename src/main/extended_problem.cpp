#include <utility>

#include <utility>

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

#include <string>
#include <unordered_set>

#include <boost/config.hpp>
#include <boost/algorithm/string.hpp>

#include <regex>

#include <glog/logging.h>

#include "extended_problem.h"

#include "util/math.h"
#include "util/datetime.h"
#include "util/json.h"
#include "forecast.h"
#include "metadata.h"

quake::ExtendedProblem::ExtendedProblem()
        : ExtendedProblem(util::DefaultPeriod(), boost::posix_time::time_duration(), std::vector<StationData>{}) {}

quake::ExtendedProblem::ExtendedProblem(boost::posix_time::time_period observation_period,
                                        boost::posix_time::time_duration switch_duration,
                                        std::vector<ExtendedProblem::StationData> station_data)
        : ExtendedProblem(observation_period, std::move(switch_duration), std::move(station_data), std::unordered_map<std::string, Forecast>{}) {}

quake::ExtendedProblem::ExtendedProblem(boost::posix_time::time_period observation_period,
                                        boost::posix_time::time_duration switch_duration,
                                        std::vector<ExtendedProblem::StationData> station_data,
                                        std::unordered_map<std::string, Forecast> forecasts)
        : ExtendedProblem(Metadata({{Metadata::Property::SwitchDuration,    switch_duration},
                                    {Metadata::Property::ObservationPeriod, observation_period}}),
                          std::move(station_data),
                          std::move(forecasts),
                          {}) {}

quake::ExtendedProblem::ExtendedProblem(Metadata metadata,
                                        std::vector<quake::ExtendedProblem::StationData> station_data,
                                        std::unordered_map<std::string, quake::Forecast> forecasts,
                                        std::unordered_map<quake::GroundStation, ExtendedProblem::StationVarModel> var_model)
        : station_data_{std::move(station_data)},
          forecasts_{std::move(forecasts)},
          metadata_{std::move(metadata)},
          var_model_{std::move(var_model)} {

    std::unordered_set<GroundStation> ground_stations;
    for (const auto &local_station_data : station_data_) {
        ground_stations.emplace(local_station_data.Station);
    }

    stations_.reserve(station_data.size());
    std::copy(std::cbegin(ground_stations), std::cend(ground_stations), std::back_inserter(stations_));
    std::sort(std::begin(stations_), std::end(stations_), [](const GroundStation &left, const GroundStation &right) -> bool {
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

    return {metadata_, std::move(rounded_station_data), forecasts_, var_model_};
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

double quake::ExtendedProblem::KeyRate(const quake::GroundStation &station,
                                       const boost::posix_time::ptime &datetime,
                                       ExtendedProblem::WeatherSample sample) const {
    return KeyRate(station, boost::posix_time::time_period{datetime, boost::posix_time::seconds(1)}, sample);
}

double zero(boost::posix_time::ptime) { return 0.0; }

double quake::ExtendedProblem::KeyRate(const quake::GroundStation &station, const boost::posix_time::time_period &period) const {
    return KeyRate(station, period, zero);
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

quake::ExtendedProblem quake::ExtendedProblem::load_json(const boost::filesystem::path &file_path) {
    std::ifstream input_stream;
    input_stream.open(file_path.string(), std::ifstream::in);
    LOG_IF(FATAL, !input_stream.is_open()) << "Failed to open file: " << file_path;

    nlohmann::json json_object;
    input_stream >> json_object;
    input_stream.close();

    ExtendedProblem problem = json_object;
    return problem;
}

std::string GetReferenceKey(quake::ExtendedProblem::WeatherSample sample) {
    switch (sample) {
        case quake::ExtendedProblem::WeatherSample::Forecast:
            return "forecast";
        case quake::ExtendedProblem::WeatherSample::Real:
            return "real";
        default:
            LOG(FATAL) << "Weather sample " << static_cast<int>(sample) << " not supported";
    }
}

const quake::Forecast &quake::ExtendedProblem::GetWeatherSample(quake::ExtendedProblem::WeatherSample sample) const {
    return forecasts_.at(GetReferenceKey(sample));
}

int forecast_scenario_comparator(const std::pair<quake::Forecast, int> &left, const std::pair<quake::Forecast, int> &right) {
    return left.second < right.second;
}

std::vector<quake::Forecast> quake::ExtendedProblem::GetWeatherSamples(quake::ExtendedProblem::WeatherSample sample) const {
    static const std::regex SCENARIO_NAME_PATTERN{"^scenario_(\\d+)$"};

    if (sample == WeatherSample::Scenario) {
        std::vector<std::pair<Forecast, int> > forecast_index_pairs;
        for (const auto &element : forecasts_) {
            std::smatch match;
            std::regex_match(element.first, match, SCENARIO_NAME_PATTERN);

            if (boost::algorithm::istarts_with(element.first, "scenario_")) {
                CHECK_GT(match.size(), 0);
                const auto scenario_number = std::stod(match[1]);
                forecast_index_pairs.emplace_back(element.second, scenario_number);
            }
        }

        std::sort(std::begin(forecast_index_pairs), std::end(forecast_index_pairs), forecast_scenario_comparator);
        std::vector<Forecast> forecasts;
        forecasts.reserve(forecast_index_pairs.size());
        for (const auto &index_pair : forecast_index_pairs) {
            forecasts.emplace_back(index_pair.first);
        }
        return forecasts;
    } else {
        const auto reference_key = GetReferenceKey(sample);
        const auto find_it = forecasts_.find(reference_key);
        if (find_it != std::cend(forecasts_)) {
            return {find_it->second};
        }
    }
}

std::vector<quake::Forecast> quake::ExtendedProblem::GetWeatherSamples(quake::ExtendedProblem::WeatherSample sample, std::size_t size) const {
    auto samples = GetWeatherSamples(sample);

    if (size > samples.size()) {
        LOG(WARNING) << "Requested more scenarios than the problem contains " << size
                     << " v.s. " << samples.size()
                     << ". The simulation will effectively use " << samples.size() << " scenarios.";
        return samples;
    }

    samples.resize(size);
    return samples;
}


void quake::to_json(nlohmann::json &json, const quake::ExtendedProblem &problem) {
    nlohmann::json object;

    object["metadata"] = problem.metadata_;
    object["stations"] = problem.station_data_;

    if (!problem.forecasts_.empty()) {
        object["forecasts"] = problem.forecasts_;
    }

    if (!problem.var_model_.empty()) {
        LOG(FATAL) << "Serialization to json of var model is not implemented";
    }

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

void quake::to_json(nlohmann::json &json, const quake::ExtendedProblem::StationVarModel &station_var_model) {}

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
    const auto metadata = json.at("metadata").get<Metadata>();
    const auto stations = json.at("stations").get<std::vector<ExtendedProblem::StationData>>();

    std::unordered_map<std::string, Forecast> forecasts;
    {
        const auto find_it = json.find("forecasts");
        if (find_it != std::end(json)) {
            forecasts = find_it->get<std::unordered_map<std::string, Forecast>>();
        }
    }

    std::unordered_map<quake::GroundStation, ExtendedProblem::StationVarModel> var_model;
    {
        const auto find_it = json.find("var_model");
        if (find_it != std::cend(json)) {
            var_model = find_it->get<std::unordered_map<quake::GroundStation, ExtendedProblem::StationVarModel> >();
        }
    }

    ExtendedProblem problem_object{metadata, stations, std::move(forecasts), std::move(var_model)};
    problem = problem_object;
}

void quake::from_json(const nlohmann::json &json, quake::ExtendedProblem::StationVarModel &station_var_model) {
    std::unordered_map<GroundStation, ExtendedProblem::StationVarModel::Parameter> parameters;
    std::unordered_map<GroundStation, double> correlations;
    for (const auto &item : json.items()) {
        auto station_or_none = GroundStation::FromNameOrNone(item.key());
        if (station_or_none != GroundStation::None) {

            const auto bundle = item.value().get<std::unordered_map<std::string, double>>();
            parameters.emplace(station_or_none, ExtendedProblem::StationVarModel::Parameter{bundle.at("L1"), bundle.at("L1.stderr")});
            correlations.emplace(station_or_none, bundle.at("corr"));
        }
    }

    auto intercept = json.at("const").get<ExtendedProblem::StationVarModel::Parameter>();
    auto residual = json.at("residual").get<ExtendedProblem::StationVarModel::Parameter>();
    auto lower_bound = json.at("lower").get<std::vector<double> >();
    auto upper_bound = json.at("upper").get<std::vector<double> >();

    station_var_model.Intercept = intercept;
    station_var_model.Residual = residual;
    station_var_model.LowerBound = lower_bound;
    station_var_model.UpperBound = upper_bound;
    station_var_model.Parameters = parameters;
    station_var_model.Correlations = correlations;
}

void quake::from_json(const nlohmann::json &json, quake::ExtendedProblem::StationVarModel::Parameter &parameter) {
    const auto value = json.at("value").get<double>();
    const auto standard_error = json.at("stderr").get<double>();

    parameter.Value = value;
    parameter.Stderr = standard_error;
}
