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

#ifndef QUAKE_SAMPLE_AVERAGE_MIP_MODEL_H
#define QUAKE_SAMPLE_AVERAGE_MIP_MODEL_H

#include "base_interval_mip_model.h"
#include "benders_callback.h"

namespace quake {

    class SampleAverageMipModel : public BaseIntervalMipModel {
    public:
        SampleAverageMipModel(quake::InferredModel const *model,
                              boost::posix_time::time_duration time_step,
                              std::vector<Forecast> forecasts,
                              double target_index);

    protected:
    public:
        double GetTrafficIndex(const Solution &solution) const override;

    protected:

        void Build(const boost::optional<Solution> &solution) override;

    private:
        class SampleAverageCallback : public BendersCallback {
        public:
            SampleAverageCallback(SampleAverageMipModel &model, double target_index, GRBVar target_distance);

        protected:
            void callback() override;

        private:
            GRBVar target_distance_var_;
            std::vector<double> scenario_distance_;
            std::vector<std::pair<std::size_t, double> > scenario_by_distance_;
            std::vector<double> dual_values_;
            std::vector<std::size_t> dual_scenarios_;
        };

        double target_index_;
        GRBVar target_distance_var_;

        std::unique_ptr<SampleAverageCallback> callback_;
    };
}


#endif //QUAKE_SAMPLE_AVERAGE_MIP_MODEL_H
