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

#ifndef QUAKE_CROSS_MOMENT_MIP_MODEL_H
#define QUAKE_CROSS_MOMENT_MIP_MODEL_H

#include "base_interval_mip_model.h"

#include <boost/config.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace quake {

    class CrossMomentMipModel : public BaseIntervalMipModel {
    public:
        CrossMomentMipModel(quake::InferredModel const *model,
                            boost::posix_time::time_duration time_step,
                            std::vector<Forecast> forecasts,
                            double target_index);

        double GetTrafficIndex(const Solution &solution) const override;

    protected:
        boost::numeric::ublas::matrix<double> ExtractScenarioMatrix(std::size_t station_index) const;

        void Build(const boost::optional<Solution> &solution) override;

    private:
        double ExpectedKeysDelivered(std::size_t station_index) const;

        bool IsSetInSolution(const BaseInterval &interval, const Solution &solution) const;

        double target_index_;
    };
}

#endif //QUAKE_CROSS_MOMENT_MIP_MODEL_H
