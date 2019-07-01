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

#ifndef QUAKE_BASE_INTERVAL_MIP_MODEL_H
#define QUAKE_BASE_INTERVAL_MIP_MODEL_H

#include <vector>

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include "forecast.h"
#include "scenario_pool.h"
#include "base_mip_model.h"

namespace quake {

    class BaseIntervalMipModel : public BaseMipModel {
    public:
        BaseIntervalMipModel(InferredModel const *model,
                             boost::posix_time::time_duration time_step,
                             std::vector<Forecast> forecasts);

        virtual double GetTrafficIndex(const Solution &solution) const = 0;

        inline std::size_t NumScenarios() const { return scenario_pool_.size(); }

        inline std::vector<VarInterval> &StationIntervals(std::size_t station_index) {
            return intervals_.at(station_index);
        }

        inline std::size_t KeysTransferred(std::size_t scenario_index, const BaseInterval &interval) const {
            return scenario_pool_.KeysTransferred(scenario_index, interval);
        }

    protected:
        void Build(const boost::optional<Solution> &solution) override;

        double GetTrafficIndex(const Solution &solution, const Forecast &forecast) const;

        virtual double GetTrafficIndexUpperBound() const;

        std::unordered_map<GroundStation,
                std::vector<boost::posix_time::time_period>> GetObservations() const override;

        ScenarioPool scenario_pool_;
        std::vector<Forecast> forecasts_;
        std::vector<std::vector<VarInterval>> intervals_;
    };
}

#endif //QUAKE_BASE_INTERVAL_MIP_MODEL_H
