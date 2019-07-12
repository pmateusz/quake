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

#ifndef QUAKE_INFERRED_MODEL_H
#define QUAKE_INFERRED_MODEL_H

#include <vector>

#include <ortools/base/basictypes.h>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#include "legacy/cp_solution.h"

#include "bounded_value.h"
#include "solution.h"
#include "forecast.h"
#include "minizinc_data_model.h"
#include "base_interval.h"

namespace quake {

    class Solution;

    class InferredModel {
    public:
        struct StageRecord {
            StageRecord(boost::posix_time::ptime start_time, std::size_t offset_index, int64 marginal_change);

            boost::posix_time::ptime StartTime;
            std::size_t OffsetIndex;
            int64 MarginalChange;
        };

        explicit InferredModel(MiniZincDataModel<int64, int64> data_model);

        static InferredModel Load(const boost::filesystem::path &path);

        std::vector<std::pair<quake::GroundStation, double> > GetAbsoluteError(const quake::CpSolution &solution) const;

        std::vector<std::pair<quake::GroundStation, double> > GetRelativeError(const quake::CpSolution &solution) const;

        int64 GetMinRateScalingFactor() const;

        inline const boost::posix_time::ptime StartTime() const { return data_model_.StartTime(); }

        inline const boost::posix_time::ptime EndTime() const { return data_model_.EndTime(); }

        inline boost::posix_time::time_period TimePeriod() const {
            return {data_model_.StartTime(), data_model_.EndTime()};
        }

        inline const std::vector<int64> &KeyRate1d() const { return key_rate_1d_; }

        inline const std::vector<int64> &KeyRateCumulative1d() const { return key_rate_cumulative_1d_; }

        inline const std::vector<std::vector<int64> > &KeyRate() const { return data_model_.KeyRate(); }

        inline bool IsWeatherAdjusted() const { return weather_adjusted_; }

        struct WeatherAdjustedCumulativeKeyRateCallback {
            WeatherAdjustedCumulativeKeyRateCallback(int ground_station, InferredModel const *model);

            WeatherAdjustedCumulativeKeyRateCallback(const WeatherAdjustedCumulativeKeyRateCallback &other);

            WeatherAdjustedCumulativeKeyRateCallback &operator=(const WeatherAdjustedCumulativeKeyRateCallback &other);

            int64 operator()(int64 time) const;

        private:
            int ground_station_;
            InferredModel const *model_;
        };

        WeatherAdjustedCumulativeKeyRateCallback GetWeatherAdjustedCumulativeKeyRate(int64 ground_station) const;

        inline const std::vector<int64> &ReconfigurationTime() const { return reconfiguration_time_; }

        inline const std::vector<int64> &TimeOffset() const { return data_model_.TimeOffset(); }

        inline const std::vector<double> &TransferShare() const { return data_model_.TransferShare(); }

        inline double TransferShare(std::size_t station_index) const {
            return data_model_.TransferShare()[station_index];
        }

        inline const std::vector<int64> &InitialBuffer() const { return initial_buffer_override_; }

        inline int64 InitialBuffer(std::size_t station_index) const {
            return initial_buffer_override_[station_index];
        }

        inline int64 KeyConsumption(std::size_t station_index) const {
            return key_consumption_override_[station_index];
        }

        inline int TimeRange() const { return data_model_.Time().size(); }

        inline int StationCount() const { return static_cast<int>(data_model_.Stations().size()); }

        inline int StationNoDummyCount() const { return static_cast<int>(data_model_.Stations().size()) - 1; }

        inline const std::vector<std::string> &Stations() const { return data_model_.Stations(); }

        inline const GroundStation Station(std::size_t station_index) const {
            const auto &name = data_model_.Stations()[station_index];
            return GroundStation::FromNameOrNone(name);
        }

        inline const std::string &DummyStation() const { return data_model_.DummyStation(); }

        inline int64 DummyStationIndex() const { return 0; }

        inline int64 SwitchDuration() const { return data_model_.SwitchDuration(); }

        inline int64 StepDuration() const { return data_model_.StepDuration(); }

        inline int64 Index(int64 station, int64 time) const { return station * station_index_multiplier_ + time; }

        int64 GetStationIndex(const GroundStation &station) const;

        inline int64 CumulativeIndex(int64 station, int64 time) const {
            return station * cumulative_station_index_multiplier_ + time;
        }

        inline int64 StationIndexMultiplier() const { return cumulative_station_index_multiplier_; };

        inline const BoundedValue<int64> &KeyRateCumulative() const {
            return key_rate_cumulative_;
        }

        inline BoundedValue<int64> &KeyRateCumulative() {
            return key_rate_cumulative_;
        }

        inline const BoundedValue<int64> &TotalKeyRate() const {
            return total_key_rate_;
        }

        inline BoundedValue<int64> &TotalKeyRate() {
            return total_key_rate_;
        }

        inline const BoundedValue<int64> &StationKeyRate() const {
            return station_key_rate_;
        }

        inline BoundedValue<int64> &StationKeyRate() {
            return station_key_rate_;
        }

        boost::posix_time::ptime Time(int64 start_index) const;

        boost::posix_time::ptime EndTime(int64 start_index, int64 duration_steps) const;

        void CheckConsistency(const CpSolution &solution) const;

        void CheckConsistency(const CpSolution::Job &job) const;

        std::vector<StageRecord> InferStages() const;

        int64 TransferredKeys(const quake::GroundStation &station,
                              const boost::posix_time::ptime &start_time,
                              const boost::posix_time::ptime &end_time) const;

        int64 WeatherAdjustedTransferredKeys(const quake::GroundStation &station,
                                             const boost::posix_time::ptime &start_time,
                                             const boost::posix_time::ptime &end_time,
                                             const Forecast &forecast) const;

        int64 WeatherAdjustedTransferredKeys(const quake::GroundStation &station,
                                             const boost::posix_time::ptime &start_time,
                                             const boost::posix_time::ptime &end_time) const;

        void Apply(const quake::Forecast &forecast);

        void Apply(const quake::Solution &previous_solution);

        int64 GetStartTransferTimeIndex(int64 station, int64 time) const;

        int64 GetStartIndex(const boost::posix_time::ptime &time) const;

        int64 GetEndIndex(const boost::posix_time::ptime &time) const;

        Solution Create(std::unordered_map<quake::GroundStation, std::vector<boost::posix_time::time_period> >) const;

    private:

        std::unordered_map<boost::gregorian::date,
                std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > LoadSunsetSunrise(
                GroundStation station) const;

        template<typename ErrorFormula>
        std::vector<std::pair<quake::GroundStation, double> > GetError(const quake::CpSolution &solution) const;

        int64 GetStartTransferTimeIndex(const CpSolution::Job &job) const;

        int64 GetEndTransferIndex(const CpSolution::Job &job) const;

        int64 WeatherAdjustedKeyRate(int64 time_index, int64 station_index, int64 cloud_cover) const;

        inline int64 WeatherAdjustedKeyRate(int64 station, int64 time) const {
            return weather_adjusted_key_rate_1d_[Index(station, time)];
        }

        inline int64 WeatherAdjustedKeyCumulativeRate(int64 station, int64 time) const {
            return weather_adjusted_key_rate_cumulative_1d_[CumulativeIndex(station, time)];
        }

        inline boost::posix_time::time_duration StepsToDuration(int64 steps) const {
            return boost::posix_time::seconds(steps * StepDuration());
        }

        MiniZincDataModel<int64, int64> data_model_;

        int64 station_index_multiplier_;
        std::vector<int64> key_rate_1d_;
        std::vector<int64> weather_adjusted_key_rate_1d_;

        int64 cumulative_station_index_multiplier_;
        std::vector<int64> key_rate_cumulative_1d_;
        std::vector<int64> weather_adjusted_key_rate_cumulative_1d_;

        std::vector<int64> key_consumption_override_;
        std::vector<int64> initial_buffer_override_;

        std::vector<int64> reconfiguration_time_;

        BoundedValue<int64> key_rate_cumulative_;
        BoundedValue<int64> total_key_rate_;
        BoundedValue<int64> station_key_rate_;

        bool weather_adjusted_;
    };
}

namespace quake {
    template<typename KeyType, typename ValueType>
    static bool
    DescendingComparer(const std::pair<KeyType, ValueType> &left, const std::pair<KeyType, ValueType> &right) {
        return std::get<1>(right) < std::get<1>(left);
    };

    template<typename ErrorFormula>
    std::vector<std::pair<quake::GroundStation, double> > InferredModel::GetError(const CpSolution &solution) const {
        static const ErrorFormula formula;

        std::vector<std::pair<quake::GroundStation, double> > errors;
        const double total_key_rate = solution.TotalKeyRate();
        CHECK_GT(total_key_rate, 0.0);
        for (std::size_t station_index = 0; station_index < StationCount(); ++station_index) {
            const double station_key_rate = solution.TransferredKeys(station_index);
            const double station_share = TransferShare(station_index);

            double error = 0.0;
            if (station_share > 0.0 && total_key_rate > 0.0) {
                const double expected_station_key_rate = station_share * total_key_rate;

                CHECK_GT(expected_station_key_rate, 0.0);
                error = formula(station_key_rate, expected_station_key_rate);
            }
            errors.emplace_back(Station(station_index), error);
        }

        std::sort(std::begin(errors),
                  std::end(errors),
                  DescendingComparer<quake::GroundStation, double>);

        return errors;
    }
}


#endif //QUAKE_INFERRED_MODEL_H
