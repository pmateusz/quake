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

#include "max_transfer_distribution_upper_bound.h"

quake::MaxTransferDistributionUpperBound::MaxTransferDistributionUpperBound(operations_research::Solver *solver,
                                                                            quake::InferredModel const *model,
                                                                            int64 other_multiplier,
                                                                            int64 max_multiplier)
        : MaxTransferUpperBound(solver, model),
          other_multiplier_{other_multiplier},
          max_multiplier_{max_multiplier} {}

void quake::MaxTransferDistributionUpperBound::Build() {
    MaxTransferUpperBound::Build();

    if (model_->TotalKeyRate().HasUpperBound()) {
        solver_->AddConstraint(solver_->MakeLessOrEqual(total_transfer_, model_->TotalKeyRate().UpperBound()));
    }

    if (model_->TotalKeyRate().HasLowerBound()) {
        solver_->AddConstraint(
                solver_->MakeGreaterOrEqual(total_transfer_, model_->TotalKeyRate().LowerBound()));
    }

    if (model_->StationKeyRate().HasUpperBound()) {
        for (auto station_index = 0; station_index < model_->StationNoDummyCount(); ++station_index) {
            solver_->AddConstraint(
                    solver_->MakeLessOrEqual(station_transfer_[station_index],
                                             model_->StationKeyRate().UpperBound()));
        }
    }

    if (model_->StationKeyRate().HasLowerBound()) {
        for (auto station_index = 0; station_index < model_->StationNoDummyCount(); ++station_index) {
            solver_->AddConstraint(
                    solver_->MakeGreaterOrEqual(station_transfer_[station_index],
                                                model_->StationKeyRate().LowerBound()));
        }
    }

    operations_research::IntExpr *const max_transfer_rate = solver_->MakeMax(station_transfer_);
    for (auto station_index = 0; station_index < model_->StationNoDummyCount(); ++station_index) {
        solver_->AddConstraint(solver_->MakeLessOrEqual(solver_->MakeProd(max_transfer_rate, max_multiplier_),
                                                        solver_->MakeProd(station_transfer_[station_index],
                                                                          other_multiplier_)));
    }
}
