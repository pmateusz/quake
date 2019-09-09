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

#ifndef QUAKE_EXTENDED_PROBLEM_H
#define QUAKE_EXTENDED_PROBLEM_H

#include <utility>
#include <vector>
#include <unordered_map>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include <nlohmann/json.hpp>

#include "util/json.h"
#include "ground_station.h"
#include "forecast.h"
#include "metadata.h"

namespace quake {

    class ExtendedProblem {
    public:
        enum class WeatherSample {
            None,
            Forecast,
            Real,
            Scenario
        };

        struct CommunicationWindowData {
            CommunicationWindowData()
                    : CommunicationWindowData(boost::posix_time::time_period(boost::posix_time::ptime(),
                                                                             boost::posix_time::seconds(0)),
                                              std::vector<double>{},
                                              std::vector<double>{}) {}

            CommunicationWindowData(boost::posix_time::time_period period,
                                    std::vector<double> elevation_angle,
                                    std::vector<double> key_rate)
                    : Period{period},
                      ElevationAngle{std::move(elevation_angle)},
                      KeyRate{std::move(key_rate)} {}

            boost::posix_time::time_period Period;
            std::vector<double> ElevationAngle;
            std::vector<double> KeyRate;
        };

        struct StationData {
            StationData()
                    : StationData(GroundStation::None, 0.0, 0, 0, std::vector<CommunicationWindowData>{}) {}

            StationData(GroundStation station,
                        double transfer_share,
                        int initial_buffer,
                        int key_consumption,
                        std::vector<CommunicationWindowData> communication_windows)
                    : Station{std::move(station)},
                      TransferShare{transfer_share},
                      InitialBuffer{initial_buffer},
                      KeyConsumption{key_consumption},
                      CommunicationWindows{std::move(communication_windows)} {}

            StationData Trim(const boost::posix_time::time_period &time_period) const;

            GroundStation Station;
            double TransferShare;
            int InitialBuffer;
            int KeyConsumption;
            std::vector<CommunicationWindowData> CommunicationWindows;
        };

        struct StationVarModel {

            struct Parameter {
                Parameter()
                        : Parameter(0.0, 0.0) {}

                Parameter(double value, double standard_error)
                        : Value{value},
                          Stderr{standard_error} {}

                double Value;
                double Stderr;
            };

            Parameter Intercept;
            Parameter Residual;
            std::unordered_map<GroundStation, Parameter> Parameters;
            std::unordered_map<GroundStation, double> Correlations;
        };

        struct MeanVarianceModel {
        public:
            using Series = SparseSeries<double>;

            struct StationMeanVarianceModel {
            public:
                StationMeanVarianceModel(const boost::posix_time::time_duration &update_frequency,
                                         const boost::posix_time::time_period &time_period,
                                         std::vector<double> mean,
                                         std::vector<double> mean_lower,
                                         std::vector<double> mean_upper,
                                         std::vector<double> variance,
                                         std::vector<double> variance_lower,
                                         std::vector<double> variance_upper)
                        : Mean{update_frequency, time_period, std::move(mean)},
                          MeanLower{update_frequency, time_period, std::move(mean_lower)},
                          MeanUpper{update_frequency, time_period, std::move(mean_upper)},
                          Variance{update_frequency, time_period, std::move(variance)},
                          VarianceLower{update_frequency, time_period, std::move(variance_lower)},
                          VarianceUpper{update_frequency, time_period, std::move(variance_upper)} {}

                StationMeanVarianceModel(Series mean,
                                         Series mean_lower,
                                         Series mean_upper,
                                         Series variance,
                                         Series variance_lower,
                                         Series variance_upper)
                        : Mean{std::move(mean)},
                          MeanLower{std::move(mean_lower)},
                          MeanUpper{std::move(mean_upper)},
                          Variance{std::move(variance)},
                          VarianceLower{std::move(variance_lower)},
                          VarianceUpper{std::move(variance_upper)} {}

                StationMeanVarianceModel Trim(const boost::posix_time::time_period &time_period) const;

                Series Mean;
                Series MeanLower;
                Series MeanUpper;
                Series Variance;
                Series VarianceLower;
                Series VarianceUpper;
            };

            MeanVarianceModel()
                    : MeanVarianceModel(0.0, {}) {}

            MeanVarianceModel(double confidence_interval,
                              std::unordered_map<GroundStation, StationMeanVarianceModel> index)
                    : ConfidenceInterval{confidence_interval},
                      Index{std::move(index)} {}

            MeanVarianceModel Trim(const boost::posix_time::time_period &time_period) const;

            double ConfidenceInterval;
            std::unordered_map<GroundStation, StationMeanVarianceModel> Index;
        };

        ExtendedProblem();

        ExtendedProblem(boost::posix_time::time_period observation_period,
                        boost::posix_time::time_duration switch_duration,
                        std::vector<StationData> station_data);

        ExtendedProblem(boost::posix_time::time_period observation_period,
                        boost::posix_time::time_duration switch_duration,
                        std::vector<StationData> station_data,
                        std::unordered_map<std::string, Forecast> forecasts);

        ExtendedProblem static load_json(const boost::filesystem::path &path);

        ExtendedProblem Round(unsigned int decimal_places) const;

        ExtendedProblem Trim(const boost::posix_time::time_period &time_period) const;

        inline const std::vector<GroundStation> &Stations() const { return stations_; }

        std::vector<boost::posix_time::time_period> TransferWindows(const GroundStation &ground_station) const;

        inline double TransferShare(const GroundStation &ground_station) const { return GetStationData(ground_station).TransferShare; }

        inline int InitialBuffer(const GroundStation &ground_station) const { return GetStationData(ground_station).InitialBuffer; }

        inline int KeyConsumption(const GroundStation &ground_station) const { return GetStationData(ground_station).KeyConsumption; }

        inline boost::posix_time::time_period ObservationPeriod() const {
            return metadata_.GetProperty<boost::posix_time::time_period>(Metadata::Property::ObservationPeriod).get();
        }

        inline boost::posix_time::time_duration SwitchDuration() const {
            return metadata_.GetProperty<boost::posix_time::time_duration>(Metadata::Property::SwitchDuration).get();
        }

        inline const Metadata &GetMetadata() const { return metadata_; }

        inline const ExtendedProblem::StationVarModel &VarModel(const GroundStation &ground_station) const { return var_model_.at(ground_station); }

        double KeyRate(const GroundStation &station, const boost::posix_time::ptime &datetime, ExtendedProblem::WeatherSample sample) const;

        double KeyRate(const GroundStation &station, const boost::posix_time::time_period &period, ExtendedProblem::WeatherSample sample) const;

        double KeyRate(const GroundStation &station, const boost::posix_time::time_period &period, const Forecast &forecast) const;

        double KeyRate(const GroundStation &station, const boost::posix_time::time_period &period) const;

        double ElevationAngle(const GroundStation &station, const boost::posix_time::ptime &datetime) const;

        const Forecast &GetWeatherSample(ExtendedProblem::WeatherSample sample) const;

        std::vector<Forecast> GetWeatherSamples(ExtendedProblem::WeatherSample sample) const;

        std::vector<Forecast> GetWeatherSamples(ExtendedProblem::WeatherSample sample, std::size_t size) const;

    private:
        ExtendedProblem(Metadata metadata,
                        std::vector<StationData> station_data,
                        std::unordered_map<std::string, Forecast> forecasts,
                        MeanVarianceModel mean_variance_model,
                        std::unordered_map<quake::GroundStation, ExtendedProblem::StationVarModel> var_model);

        double KeyRate(const GroundStation &station,
                       const boost::posix_time::time_period &period,
                       const std::function<double(boost::posix_time::ptime)> &weather_callback) const;

        const StationData &GetStationData(const quake::GroundStation &station) const;

        friend void to_json(nlohmann::json &json, const ExtendedProblem &problem);

        friend void from_json(const nlohmann::json &json, ExtendedProblem &problem);

        std::vector<StationData> station_data_;
        std::unordered_map<std::string, Forecast> forecasts_;

        Metadata metadata_;

        std::vector<GroundStation> stations_;
        MeanVarianceModel mean_variance_model_;
        std::unordered_map<GroundStation, ExtendedProblem::StationVarModel> var_model_;
    };

    void from_json(const nlohmann::json &json, ExtendedProblem &problem);

    void from_json(const nlohmann::json &json, ExtendedProblem::StationData &station_data);

    void from_json(const nlohmann::json &json, ExtendedProblem::CommunicationWindowData &communication_window_data);

    void from_json(const nlohmann::json &json, ExtendedProblem::StationVarModel &station_var_model);

    void from_json(const nlohmann::json &json, ExtendedProblem::StationVarModel::Parameter &parameter);

    void from_json(const nlohmann::json &json, ExtendedProblem::MeanVarianceModel &mean_variance_model);

    void to_json(nlohmann::json &json, const ExtendedProblem &problem);

    void to_json(nlohmann::json &json, const ExtendedProblem::StationData &station_data);

    void to_json(nlohmann::json &json, const ExtendedProblem::CommunicationWindowData &communication_window_data);

    void to_json(nlohmann::json &json, const ExtendedProblem::StationVarModel &station_var_model);
}


#endif //QUAKE_EXTENDED_PROBLEM_H
