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

#ifndef QUAKE_WORST_CASE_MIP_MODEL_H
#define QUAKE_WORST_CASE_MIP_MODEL_H

#include <vector>

#include <gurobi_c++.h>

#include "base_interval_mip_model.h"

namespace quake {

    class WorstCaseMipModel : public BaseIntervalMipModel {
    public:
        WorstCaseMipModel(quake::InferredModel const *model,
                          boost::posix_time::time_duration time_step,
                          std::vector<Forecast> forecasts);

        double GetTrafficIndex(const Solution &solution) const override;

    protected:
        void Build(const boost::optional<Solution> &solution) override;
    };
}


#endif //QUAKE_WORST_CASE_MIP_MODEL_H
