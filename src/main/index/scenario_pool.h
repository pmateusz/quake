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

#ifndef QUAKE_SCENARIO_POOL_H
#define QUAKE_SCENARIO_POOL_H

#include <vector>
#include <unordered_map>

#include <ortools/base/basictypes.h>

#include "base_interval.h"
#include "legacy/inferred_model.h"

namespace quake {

    class Forecast;

    class ScenarioPool {
    public:
        ScenarioPool();

        ScenarioPool(const ScenarioPool &other);

        ScenarioPool(ScenarioPool &&other) noexcept;

        ScenarioPool &operator=(const ScenarioPool &other);

        ScenarioPool &operator=(ScenarioPool &&other) noexcept;

        ScenarioPool(const std::vector<std::vector<quake::VarInterval> > &intervals,
                     const std::vector<Forecast> &forecasts,
                     quake::InferredModel const *model);

        inline std::size_t size() const { return num_scenarios_; }

        int64 KeysTransferred(std::size_t scenario, const BaseInterval &interval) const;

        int64 MeanKeysTransferred(const BaseInterval &interval) const;

    private:
        quake::InferredModel const *model_;
        std::unordered_map<BaseInterval, std::vector<int64> > transferred_keys_index_;
        std::size_t num_scenarios_;
    };
}


#endif //QUAKE_SCENARIO_POOL_H
