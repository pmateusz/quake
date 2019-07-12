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

#ifndef QUAKE_SOLUTION_H
#define QUAKE_SOLUTION_H

#include <unordered_map>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#include <ortools/base/basictypes.h>

#include <nlohmann/json.hpp>

#include "ground_station.h"

namespace quake {

    class Solution {
    public:
        Solution();

        explicit Solution(std::unordered_map<GroundStation, int64> final_buffers);

        Solution(std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period> > observations,
                 std::unordered_map<GroundStation, int64> final_buffers);

        static Solution load_json(const boost::filesystem::path &path);

        inline const std::unordered_map<GroundStation, int64> &FinalBuffers() const { return final_buffers_; }

        std::vector<GroundStation> Stations() const;

        int64 FinalBuffer(const GroundStation &station) const;

        const std::vector<boost::posix_time::time_period> &ObservationWindows(const GroundStation &station) const;

    private:
        friend void to_json(nlohmann::json &json, const Solution &solution);

        static const std::vector<boost::posix_time::time_period> NO_OBSERVATIONS;

        std::unordered_map<GroundStation, std::vector<boost::posix_time::time_period> > observations_;
        std::unordered_map<GroundStation, int64> final_buffers_;
    };

    void to_json(nlohmann::json &json, const Solution &solution);

    void from_json(const nlohmann::json &json, Solution &solution);
}


#endif //QUAKE_SOLUTION_H
