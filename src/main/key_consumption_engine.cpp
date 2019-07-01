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

#include "key_consumption_engine.h"

#include <ortools/linear_solver/linear_solver.h>

std::vector<std::vector<int64> > quake::MaxFlowEngine::ComputeFlow(std::vector<int64> capacity) {
    operations_research::MPSolver solver("LinearProgramming",
                                         operations_research::MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);

    const auto capacity_size = capacity.size();
    std::vector<std::vector<operations_research::MPVariable *> > flow_matrix(capacity_size,
                                                                             std::vector<operations_research::MPVariable *>(
                                                                                     capacity_size, nullptr));

    const double infinity = solver.infinity();
    for (auto source_index = 0; source_index < capacity_size; ++source_index) {
        for (auto target_index = source_index + 1; target_index < capacity_size; ++target_index) {
            if (source_index == target_index) {
                continue;
            }

            const auto max_capacity = std::min(capacity[source_index], capacity[target_index]);
            std::stringstream variable_name;
            variable_name << source_index << '-' << target_index;
            operations_research::MPVariable *const flow_variable
                    = solver.MakeIntVar(0, max_capacity, variable_name.str());
            flow_matrix[source_index][target_index] = flow_variable;
            flow_matrix[target_index][source_index] = flow_variable;
        }
    }

    for (auto source_index = 0; source_index < capacity_size; ++source_index) {
        for (auto target_index = source_index + 1; target_index < capacity_size; ++target_index) {
            if (source_index != target_index) {
                CHECK(flow_matrix[source_index][target_index] != nullptr);
            } else {
                CHECK(flow_matrix[source_index][target_index] == nullptr);
            }
        }
    }

    operations_research::MPObjective *const objective = solver.MutableObjective();
    for (auto source_index = 0; source_index < capacity_size; ++source_index) {
        for (auto target_index = source_index + 1; target_index < capacity_size; ++target_index) {
            operations_research::MPConstraint *const is_non_negative = solver.MakeRowConstraint(0.0, infinity);
            operations_research::MPVariable *const flow_variable = flow_matrix[source_index][target_index];
            CHECK_NOTNULL(flow_variable);
            is_non_negative->SetCoefficient(flow_variable, 1.0);
            objective->SetCoefficient(flow_variable, 1.0);
        }
    }
    objective->SetMaximization();

    for (auto source_index = 0; source_index < capacity_size; ++source_index) {
        operations_research::MPConstraint *const is_within_capacity = solver.MakeRowConstraint(0.0,
                                                                                               capacity[source_index]);
        for (auto target_index = 0; target_index < capacity_size; ++target_index) {
            if (source_index == target_index) {
                continue;
            }

            operations_research::MPVariable *const flow_variable = flow_matrix[source_index][target_index];
            CHECK_NOTNULL(flow_variable);
            is_within_capacity->SetCoefficient(flow_variable, 1.0);
        }
    }

    const operations_research::MPSolver::ResultStatus result_status = solver.Solve();
    if (result_status != operations_research::MPSolver::OPTIMAL) {
        LOG(FATAL) << "The problem does not have an optimal solution!";
    }

    std::vector<std::vector<int64> > optimal_flow(capacity_size, std::vector<int64>(capacity_size, 0));
    for (auto source_index = 0; source_index < capacity_size; ++source_index) {
        for (auto target_index = source_index + 1; target_index < capacity_size; ++target_index) {
            const auto flow_element = static_cast<int64>(flow_matrix[source_index][target_index]->solution_value());
            optimal_flow[source_index][target_index] = flow_element;
            optimal_flow[target_index][source_index] = flow_element;
        }
    }
    return optimal_flow;
}
