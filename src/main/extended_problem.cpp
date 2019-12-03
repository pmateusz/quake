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

#include <boost/algorithm/string.hpp>

#include <regex>

#include <glog/logging.h>

#include "extended_problem.h"

#include "util/math.h"
#include "util/datetime.h"
#include "util/json.h"
#include "forecast.h"
#include "metadata.h"
#include "solution.h"


quake::ExtendedProblem::StationData quake::ExtendedProblem::StationData::Trim(const boost::posix_time::time_period &time_period) const {
    std::vector<CommunicationWindowData> trimmed_communication_windows;

    for (const auto &window : CommunicationWindows) {
        if (window.Period.is_before(time_period.begin())) { continue; }
        if (window.Period.is_after(time_period.end())) { continue; }

        if (time_period.contains(window.Period)) {
            trimmed_communication_windows.emplace_back(window);
            continue;
        }

        CHECK(!window.Period.intersects(time_period)) << "Case when communication window intersects with time_period is not implemented";

        LOG(FATAL) << "Unexpected case";
    }

    return {Station, TransferShare, InitialBuffer, KeyConsumption, std::move(trimmed_communication_windows)};
}


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
                          boost::none,
                          {}) {}

quake::ExtendedProblem::ExtendedProblem(Metadata metadata,
                                        std::vector<quake::ExtendedProblem::StationData> station_data,
                                        std::unordered_map<std::string, quake::Forecast> forecasts,
                                        boost::optional<MeanVarianceModel> mean_variance_model,
                                        std::unordered_map<quake::GroundStation, ExtendedProblem::StationVarModel> var_model)
        : station_data_{std::move(station_data)},
          forecasts_{std::move(forecasts)},
          metadata_{std::move(metadata)},
          mean_variance_model_{std::move(mean_variance_model)},
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

quake::ExtendedProblem quake::ExtendedProblem::Trim(const boost::posix_time::time_period &time_period) const {
    std::vector<StationData> trimmed_station_data;
    for (const auto &station_data_row: station_data_) {
        trimmed_station_data.emplace_back(station_data_row.Trim(time_period));
    }

    std::unordered_map<std::string, Forecast> trimmed_forecast;
    for (const auto &forecast_entry: forecasts_) {
        trimmed_forecast.emplace(forecast_entry.first, forecast_entry.second.Trim(time_period));
    }

    boost::optional<MeanVarianceModel> trimmed_mean_variance_model;
    if (mean_variance_model_) {
        trimmed_mean_variance_model = mean_variance_model_->Trim(time_period);
    }

    return {metadata_.Trim(time_period),
            std::move(trimmed_station_data),
            std::move(trimmed_forecast),
            std::move(trimmed_mean_variance_model),
            var_model_};
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
                                          util::round(station_data.TransferShare, decimal_places),
                                          station_data.InitialBuffer,
                                          station_data.KeyConsumption,
                                          std::move(communication_window_data));
    }

    return {metadata_, std::move(rounded_station_data), forecasts_, mean_variance_model_, var_model_};
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

quake::ExtendedProblem::StationData &quake::ExtendedProblem::GetStationData(const quake::GroundStation &station) {
    for (auto &station_data: station_data_) {
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

    auto forecast_series_size = 0;
    if (!forecasts.empty()) {
        const auto forecast_begin_it = std::cbegin(forecasts);
        const auto forecast_end_it = std::cend(forecasts);

        const std::unordered_map<quake::GroundStation, Forecast::Series> &first_index = forecast_begin_it->second.Index();
        const auto first_index_it = std::cbegin(first_index);
        if (first_index_it != std::cend(first_index)) {
            forecast_series_size = first_index_it->second.Values().size();
        }

        for (auto forecast_it = forecast_begin_it; forecast_it != forecast_end_it; ++forecast_it) {
            for (const auto &station_entry : forecast_it->second.Index()) {
                CHECK_EQ(station_entry.second.Values().size(), forecast_series_size);
            }
        }
    }

    boost::optional<quake::ExtendedProblem::MeanVarianceModel> mean_variance_model_opt;
    const auto mean_variance_model_it = json.find("mean_variance_model");
    if (mean_variance_model_it != std::cend(json)) {
        mean_variance_model_opt = mean_variance_model_it->get<ExtendedProblem::MeanVarianceModel>();

        if (!forecasts.empty()) {
            const auto &first_forecast = forecasts.begin()->second;
            const auto &first_forecast_series = first_forecast.Index().begin()->second;
            for (const auto &entry : mean_variance_model_opt->Index) {
                CHECK_EQ(entry.second.Mean.Period(), first_forecast_series.Period());
                CHECK_EQ(entry.second.Mean.UpdateFrequency(), first_forecast_series.UpdateFrequency());
            }
        }
    }

    problem = {metadata, stations, std::move(forecasts), std::move(mean_variance_model_opt), std::move(var_model)};
}

void quake::ExtendedProblem::OverwriteInitialConditions(const Solution &previous_solution) {
    for (auto &current_station_data : station_data_) {
        current_station_data.InitialBuffer = previous_solution.FinalBuffer(current_station_data.Station);
    }

    if (TrackConsumption()) {
        auto min_consumption_index = std::numeric_limits<double>::max();
        for (auto &current_station_data : station_data_) {
            double current_consumption_index = static_cast<double>(current_station_data.InitialBuffer) / current_station_data.TransferShare;
            min_consumption_index = std::min(min_consumption_index, current_consumption_index);
        }

        const auto total_days = ObservationPeriod().length().total_seconds() / boost::posix_time::hours(24).total_seconds();
        for (auto &current_station_data : station_data_) {
            current_station_data.KeyConsumption = static_cast<int>(std::floor(min_consumption_index * current_station_data.TransferShare
                                                                              / static_cast<double>(total_days)));
        }
    }
}

bool quake::ExtendedProblem::TrackConsumption() const {
    for (const auto &current_station_data : station_data_) {
        if (current_station_data.KeyConsumption > 0) {
            return true;
        }
    }
    return false;
}

double quake::ExtendedProblem::GetTrafficIndex(const std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period>> &observations,
                                               const Forecast &forecast) const {
    std::unordered_map<GroundStation, double> traffic_index;
    for (const auto &station : this->Stations()) {
        traffic_index[station] = this->InitialBuffer(station);
    }

    for (const auto &observation_group : observations) {
        for (const auto &observation : observation_group.second) {
            traffic_index[observation_group.first] += this->KeyRate(observation_group.first, observation, forecast);
        }
    }

    for (const auto &station : this->Stations()) {
        traffic_index[station] /= this->TransferShare(station);
    }

    auto output_traffic_index = std::numeric_limits<double>::max();
    for (const auto &station : this->Stations()) {
        output_traffic_index = std::min(output_traffic_index, traffic_index[station]);
    }
    return output_traffic_index;
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

    station_var_model.Intercept = intercept;
    station_var_model.Residual = residual;
    station_var_model.Parameters = parameters;
    station_var_model.Correlations = correlations;
}

void quake::from_json(const nlohmann::json &json, quake::ExtendedProblem::StationVarModel::Parameter &parameter) {
    const auto value = json.at("value").get<double>();
    const auto standard_error = json.at("stderr").get<double>();

    parameter.Value = value;
    parameter.Stderr = standard_error;
}

void quake::from_json(const nlohmann::json &json, quake::ExtendedProblem::MeanVarianceModel &mean_variance_model) {
    const auto confidence_interval = json.at("confidence").get<double>();
    const auto time_index = json.at("index").get<std::vector<boost::posix_time::ptime> >();

    boost::posix_time::time_period time_period(boost::posix_time::min_date_time,
                                               static_cast<boost::posix_time::ptime>(boost::posix_time::max_date_time));
    boost::posix_time::time_duration update_frequency;

    if (!time_index.empty()) {
        if (time_index.size() >= 2) {
            update_frequency = time_index[1] - time_index[0];

            const auto num_updates = time_index.size();
            for (auto update_index = 1; update_index < num_updates; ++update_index) {
                CHECK_EQ(time_index[update_index] - time_index[update_index - 1], update_frequency);
            }
        }

        time_period = {time_index.front(), time_index.back() + update_frequency}; // the last time period is not included
    }

    const auto size = time_index.size();
    std::unordered_map<GroundStation, quake::ExtendedProblem::MeanVarianceModel::StationMeanVarianceModel> data;
    for (const auto &json_entry : json.items()) {
        const auto &ground_station = GroundStation::FromNameOrNone(json_entry.key());
        if (ground_station == GroundStation::None) { continue; }

        const auto mean = json_entry.value().at("mean").get<std::vector<double>>();
        CHECK_EQ(mean.size(), size);

        const auto mean_lower = json_entry.value().at("mean_lower").get<std::vector<double>>();
        CHECK_EQ(mean_lower.size(), size);

        const auto mean_upper = json_entry.value().at("mean_upper").get<std::vector<double>>();
        CHECK_EQ(mean_upper.size(), size);

        const auto variance = json_entry.value().at("variance").get<std::vector<double>>();
        CHECK_EQ(variance.size(), size);

        const auto variance_lower = json_entry.value().at("variance_lower").get<std::vector<double>>();
        CHECK_EQ(variance_lower.size(), size);

        const auto variance_upper = json_entry.value().at("variance_upper").get<std::vector<double>>();
        CHECK_EQ(variance_upper.size(), size);

        for (auto pos = 0; pos < size; ++pos) {
            CHECK_LE(mean_lower[pos], mean_upper[pos]);
            if (mean_lower[pos] <= 100.0) {
                CHECK_LE(mean_lower[pos], mean[pos]);
            }
            if (mean_upper[pos] >= 0.0) {
                CHECK_LE(mean[pos], mean_upper[pos]);
            }

            CHECK_LE(variance_lower[pos], variance[pos]);
            CHECK_LE(variance[pos], variance_upper[pos]);
        }

        data.emplace(ground_station, quake::ExtendedProblem::MeanVarianceModel::StationMeanVarianceModel{
                quake::ExtendedProblem::MeanVarianceModel::Series(update_frequency, time_period, mean),
                quake::ExtendedProblem::MeanVarianceModel::Series(update_frequency, time_period, mean_lower),
                quake::ExtendedProblem::MeanVarianceModel::Series(update_frequency, time_period, mean_upper),
                quake::ExtendedProblem::MeanVarianceModel::Series(update_frequency, time_period, variance),
                quake::ExtendedProblem::MeanVarianceModel::Series(update_frequency, time_period, variance_lower),
                quake::ExtendedProblem::MeanVarianceModel::Series(update_frequency, time_period, variance_upper)});
    }

    mean_variance_model = {confidence_interval, std::move(data)};
}

quake::ExtendedProblem::MeanVarianceModel quake::ExtendedProblem::MeanVarianceModel::Trim(const boost::posix_time::time_period &time_period) const {
    std::unordered_map<GroundStation, StationMeanVarianceModel> trimmed_index;
    for (const auto &entry : Index) {
        trimmed_index.emplace(entry.first, entry.second.Trim(time_period));
    }
    return {ConfidenceInterval, std::move(trimmed_index)};
}

quake::ExtendedProblem::MeanVarianceModel::StationMeanVarianceModel quake::ExtendedProblem::MeanVarianceModel::StationMeanVarianceModel::Trim(
        const boost::posix_time::time_period &time_period) const {
    return {Mean.Trim(time_period), MeanLower.Trim(time_period), MeanUpper.Trim(time_period),
            Variance.Trim(time_period), VarianceLower.Trim(time_period), VarianceUpper.Trim(time_period)};
}
