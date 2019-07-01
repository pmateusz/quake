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

#ifndef QUAKE_CP_SOLUTION_JSON_READER_H
#define QUAKE_CP_SOLUTION_JSON_READER_H

#include "solution.h"

#include <nlohmann/json.hpp>

namespace quake {

    template<typename StreamType>
    class SolutionJsonReader {
    public:
        explicit SolutionJsonReader(StreamType stream) noexcept;

        SolutionJsonReader(const SolutionJsonReader &other) = delete;

        SolutionJsonReader &operator=(const SolutionJsonReader &other) = delete;

        SolutionJsonReader(SolutionJsonReader &&other) noexcept;

        SolutionJsonReader &operator=(SolutionJsonReader &&other) noexcept;

        Solution Read();

        void Close();

        virtual ~SolutionJsonReader();

    private:
        void TryClose();

        StreamType stream_;
    };
}

namespace quake {

    template<typename StreamType>
    SolutionJsonReader<StreamType>::SolutionJsonReader(StreamType stream) noexcept
            : stream_{std::move(stream)} {}

    template<typename StreamType>
    SolutionJsonReader<StreamType>::SolutionJsonReader(SolutionJsonReader<StreamType> &&other) noexcept
            : stream_{std::move(other)} {}

    template<typename StreamType>
    SolutionJsonReader<StreamType> &SolutionJsonReader<StreamType>::operator=(
            SolutionJsonReader &&other) noexcept {
        stream_ = std::move(other.stream_);
        return *this;
    }

    template<typename StreamType>
    void SolutionJsonReader<StreamType>::Close() {
        TryClose();
    }

    template<typename StreamType>
    void SolutionJsonReader<StreamType>::TryClose() {
        if (stream_.is_open()) {
            stream_.close();
        }
    }

    template<typename StreamType>
    SolutionJsonReader<StreamType>::~SolutionJsonReader() {
        TryClose();
    }

    template<typename StreamType>
    Solution SolutionJsonReader<StreamType>::Read() {
        nlohmann::json json_object;
        stream_ >> json_object;

        const auto solutions_it = json_object.find("solutions");
        LOG_IF(FATAL, solutions_it == json_object.end()) << "No solutions found";
        CHECK(solutions_it->is_array());
        CHECK(!solutions_it->empty());

        std::unordered_map<GroundStation, int64> final_buffer_rates;
        nlohmann::json first_solution = (*solutions_it)[0];
        for (const auto &station : first_solution["stations"]) {
            const auto station_name = station["station"].get<std::string>();
            const auto ground_station = GroundStation::FromNameOrNone(station_name);

            if (ground_station == GroundStation::None) { continue; }

            auto final_buffer_rate = 0;
            if (station.find("final_buffer") != station.end()) {
                final_buffer_rate = station["final_buffer"].get<int64>();
            }
            final_buffer_rates.emplace(ground_station, final_buffer_rate);
        }

        return Solution{std::move(final_buffer_rates)};
    }
}

#endif //QUAKE_CP_SOLUTION_JSON_READER_H
