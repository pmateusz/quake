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

#include <glog/logging.h>

#include "lns_path_stage_operator.h"

quake::LNSPathStageOperator::LNSPathStageOperator(const std::vector<std::vector<operations_research::IntVar *> > &next,
                                                  const std::vector<std::vector<operations_research::IntVar *> > &start)
        : BaseLns(Flatten(next)),
          current_path_{0},
          current_node_{0},
          first_run_{true} {
    AddVars(Flatten(start));

    CHECK_EQ(next.size(), start.size());
    paths_ = next.size();
    for (auto path_index = 0; path_index < paths_; ++path_index) {
        CHECK_EQ(next[path_index].size() + 1, start[path_index].size());

    }

    std::size_t current_offset = 0;
    for (auto path_index = 0; path_index < paths_; ++path_index) {
        auto path_size = next[path_index].size();
        size_.push_back(path_size);
        next_offset_.push_back(current_offset);
        current_offset += path_size;
    }

    for (auto path_index = 0; path_index < paths_; ++path_index) {
        start_offset_.push_back(current_offset);
        current_offset += start[path_index].size();
    }
}

void quake::LNSPathStageOperator::InitFragments() {
    inactive_node_.clear();
    end_node_.clear();
    min_node_ = kint64max;
    max_node_ = kint64min;

    for (auto path_index = 0; path_index < paths_; ++path_index) {
        std::vector<std::size_t> inactive_node;
        std::vector<std::size_t> prev_node(size_[path_index], 0);

        std::size_t end_node = 0;
        for (std::size_t node = 0; node < size_[path_index]; ++node) {
            int64 node_index = next_offset_[path_index] + node;
            int64 next_node = Value(node_index);
            if (next_node == node) {
                inactive_node.push_back(node);
            } else if (next_node >= size_[path_index]) {
                end_node = node;
            }

            if (next_node < size_[path_index]) {
                min_node_ = std::min(min_node_, next_node);
                max_node_ = std::max(max_node_, next_node);
                prev_node[next_node] = node;
            }
        }

        inactive_node_.emplace_back(std::move(inactive_node));
        end_node_.push_back(end_node);
    }

    if (first_run_) {
        initial_path_location_.clear();

        for (auto path_index = 0; path_index < paths_; ++path_index) {
            std::vector<std::size_t> initial_path_location(size_[path_index], 0);
            for (std::size_t node = 0; node < size_[path_index]; ++node) {
                int64 node_index = next_offset_[path_index] + node;
                int64 next_node = Value(node_index);
                if (next_node < size_[path_index]) {
                    initial_path_location[next_node] = node;
                }
            }

            initial_path_location_.push_back(initial_path_location);
        }

        first_run_ = false;
    }

    current_path_ = 0;
    current_inner_path_ = 1;
    current_node_ = min_node_;
}

bool quake::LNSPathStageOperator::NextFragment() {
    if (current_node_ > max_node_ || current_path_ >= paths_) {
        return false;
    }

    ReleaseChain(current_path_, initial_path_location_[current_path_][current_node_], 2);
    ReleaseInactive(current_path_);
    if (current_inner_path_ < paths_) {
        ReleaseChain(current_inner_path_, initial_path_location_[current_inner_path_][current_node_], 2);
        ReleaseInactive(current_inner_path_);
    }

    NextIteration();

    return true;
}

void quake::LNSPathStageOperator::NextIteration() {
    if (current_inner_path_ < paths_ - 1) {
        ++current_inner_path_;
        return;
    }

    if (current_path_ < paths_ - 2) {
        ++current_path_;
        current_inner_path_ = current_path_ + 1;
        return;
    }


    ++current_node_;
    current_path_ = 0;
    current_inner_path_ = current_path_ + 1;
}

bool quake::LNSPathStageOperator::ReleaseInactive(std::size_t path) {
    if (inactive_node_[path].empty()) {
        return false;
    }

    for (auto node : inactive_node_[path]) {
        ReleaseNode(path, node);
    }

    return true;
}

bool quake::LNSPathStageOperator::ReleaseChain(std::size_t path, std::size_t start_node, std::size_t max_length) {
    auto length = 0;
    std::size_t current_node = start_node;
    for (; current_node < size_[path] && length < max_length; ++length) {
        auto next_node = static_cast<std::size_t>(OldValue(Next(path, current_node)));
        ReleaseNode(path, current_node);
        current_node = next_node;
    }
    return length > 0;
}

void quake::LNSPathStageOperator::ReleaseNode(std::size_t path, std::size_t node) {
    Deactivate(Next(path, node));
    Deactivate(Start(path, node));
}

int64 quake::LNSPathStageOperator::Next(std::size_t path, std::size_t node) {
    return static_cast<int64>(next_offset_[path] + node);
}

int64 quake::LNSPathStageOperator::Start(std::size_t path, std::size_t node) {
    return static_cast<int64>(start_offset_[path] + node);
}

std::vector<operations_research::IntVar *> quake::LNSPathStageOperator::Flatten(
        std::vector<std::vector<operations_research::IntVar *> > vars) {
    std::vector<operations_research::IntVar *> result;
    for (const auto &row : vars) {
        std::copy(std::cbegin(row), std::cend(row), std::back_inserter(result));
    }
    return result;
}




