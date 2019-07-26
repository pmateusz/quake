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

#ifndef QUAKE_FORECAST_H
#define QUAKE_FORECAST_H

#include <string>
#include <vector>
#include <unordered_map>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#include <nlohmann/json.hpp>

#include <ortools/base/basictypes.h>

#include "ground_station.h"

namespace quake {

    class Forecast {
    public:
        template<typename ValueType>
        class SparseSeries {
        public:
            SparseSeries(const SparseSeries<ValueType> &other)
                    : update_frequency_{other.update_frequency_},
                      period_{other.period_},
                      values_{other.values_} {}

            SparseSeries(SparseSeries<ValueType> &&other) noexcept
                    : update_frequency_{std::move(other.update_frequency_)},
                      period_{std::move(other.period_)},
                      values_{std::move(other.values_)} {}

            SparseSeries(boost::posix_time::time_duration update_frequency,
                         boost::posix_time::time_period period,
                         std::vector<ValueType> values)
                    : update_frequency_{std::move(update_frequency)},
                      period_{period},
                      values_{std::move(values)} {}

            inline const std::vector<ValueType> &Values() const { return values_; }

            inline const boost::posix_time::time_period &Period() const { return period_; }

            inline const boost::posix_time::time_duration &UpdateFrequency() const { return update_frequency_; }

            ValueType operator()(const boost::posix_time::ptime &value) const {
                if (!period_.contains(value) && period_.end() != value) {
                    LOG(WARNING) << "Time " << value << " is outside the period of observations " << period_ << ".";
                    return 0;
                }

                const auto time_from_start = value - period_.begin();
                CHECK(!time_from_start.is_negative());

                const auto left_index = static_cast<std::size_t>(time_from_start.total_seconds() / update_frequency_.total_seconds());

                // old behaviour
//                const auto remaining_seconds = time_from_start.total_seconds() - left_index * update_frequency_.total_seconds();
//                if (remaining_seconds > update_frequency_.total_seconds() / 2) {
//                    const auto right_index = left_index + 1;
//                    if (right_index < values_.size()) {
//                        LOG(INFO) << "Right index: " << right_index;
//                        return values_.at(right_index);
//                    }
//                }

                return values_.at(left_index);
            }

        private:
            boost::posix_time::time_duration update_frequency_;
            boost::posix_time::time_period period_;
            std::vector<ValueType> values_;
        };

        using SeriesValueType = double;
        using Series = SparseSeries<SeriesValueType>;

        static Forecast load_csv(const boost::filesystem::path &input_file);

        Forecast();

        explicit Forecast(std::unordered_map<quake::GroundStation, Series> index);

        SeriesValueType GetCloudCover(const quake::GroundStation &station, const boost::posix_time::ptime &time) const;

        inline const std::unordered_map<quake::GroundStation, Series> &Index() const { return index_; }

    private:
        friend void to_json(nlohmann::json &json, const Forecast &forecast);

        std::unordered_map<quake::GroundStation, Series> index_;
    };

    void from_json(const nlohmann::json &json, Forecast &forecast);

    void to_json(nlohmann::json &json, const Forecast &forecast);
}
#endif //QUAKE_FORECAST_H
