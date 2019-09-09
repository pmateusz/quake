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
#include "sparse_series.h"

namespace quake {

    class Forecast {
    public:
        using SeriesValueType = double;
        using Series = SparseSeries<SeriesValueType>;

        static Forecast load_csv(const boost::filesystem::path &input_file);

        Forecast();

        explicit Forecast(std::unordered_map<quake::GroundStation, Series> index);

        Forecast Trim(const boost::posix_time::time_period &time_period) const;

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
