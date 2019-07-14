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

#ifndef QUAKE_CVAR_MIP_MODEL_H
#define QUAKE_CVAR_MIP_MODEL_H

#include <memory>

#include "base_interval_mip_model.h"
#include "benders_callback.h"

namespace quake {

    class CVarMipModel : public BaseIntervalMipModel {
    public:
        CVarMipModel(ExtendedProblem const *problem,
                     boost::posix_time::time_duration time_step,
                     std::vector<Forecast> forecasts,
                     double target_index,
                     double epsilon);

    protected:
        void Build(const boost::optional<Solution> &solution) override;

    private:
        class CVarCallback : public BendersCallback {
        public:
            CVarCallback(CVarMipModel &model, double target_index, double epsilon);

        protected:
            void callback() override;

        private:
            double epsilon_;
        };

        double target_index_;
        double epsilon_;

        std::unique_ptr<CVarCallback> callback_;
    };
}


#endif //QUAKE_CVAR_MIP_MODEL_H
