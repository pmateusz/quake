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

#ifndef QUAKE_CP_SOLUTION_JSON_WRITER_H
#define QUAKE_CP_SOLUTION_JSON_WRITER_H

#include <utility>

#include <nlohmann/json.hpp>

#include "cp_solution.h"
#include "inferred_model.h"
#include "util/json.h"

namespace quake {

    template<typename StreamType>
    class CpSolutionJsonWriter {
    public:
        explicit CpSolutionJsonWriter(StreamType stream) noexcept;

        CpSolutionJsonWriter(const CpSolutionJsonWriter &other) = delete;

        CpSolutionJsonWriter &operator=(const CpSolutionJsonWriter &other) = delete;

        CpSolutionJsonWriter(CpSolutionJsonWriter &&other) noexcept;

        CpSolutionJsonWriter &operator=(CpSolutionJsonWriter &&other) noexcept;

        void Write(const InferredModel &model, const CpSolution &solution);

        void Write(const InferredModel &model, const std::vector<CpSolution> &solutions);

        virtual ~CpSolutionJsonWriter();

        void Close();

    private:
        nlohmann::json ToJson(const InferredModel &model, const CpSolution &solution) const;

        nlohmann::json ToJson(const InferredModel &model) const;

        void TryClose();

        StreamType stream_;
    };
}

namespace quake {

    template<typename StreamType>
    CpSolutionJsonWriter<StreamType>::CpSolutionJsonWriter(StreamType stream) noexcept
            : stream_{std::move(stream)} {}

    template<typename StreamType>
    CpSolutionJsonWriter<StreamType>::CpSolutionJsonWriter(CpSolutionJsonWriter &&other) noexcept
            : stream_{std::move(other)} {}

    template<typename StreamType>
    CpSolutionJsonWriter<StreamType> &CpSolutionJsonWriter<StreamType>::operator=(
            CpSolutionJsonWriter &&other) noexcept {
        stream_ = std::move(other.stream_);
        return *this;
    }

    template<typename StreamType>
    CpSolutionJsonWriter<StreamType>::~CpSolutionJsonWriter() {
        TryClose();
    }

    template<typename StreamType>
    void CpSolutionJsonWriter<StreamType>::Close() {
        TryClose();
    }

    template<typename StreamType>
    void CpSolutionJsonWriter<StreamType>::TryClose() {
        if (stream_.is_open()) {
            stream_.close();
        }
    }

    template<typename StreamType>
    nlohmann::json CpSolutionJsonWriter<StreamType>::ToJson(const InferredModel &model,
                                                            const CpSolution &solution) const {
        std::vector<nlohmann::json> stations_body;

        const auto &cumulative_key_rates = solution.TransferredKeys();
        const auto &final_buffer = solution.FinalBuffer();
        const auto &stations = model.Stations();
        for (auto station_index = 0; station_index < stations.size(); ++station_index) {
            stations_body.push_back({
                                            {"station",      stations[station_index]},
                                            {"key_rate",     cumulative_key_rates[station_index]},
                                            {"final_buffer", final_buffer[station_index]},
                                    });
        }

        std::vector<nlohmann::json> actions_body;
        for (const auto &action: solution.Jobs()) {
            if (action.Start() >= model.TimeRange()) {
                continue;
            }
            const auto key_rate = action.KeysTransferred();
            const auto start_time = model.Time(action.Start());
            const auto end_time = model.EndTime(action.Start(), action.Duration());

            CHECK_GE(key_rate, 0);
            CHECK_LE(start_time, end_time);
            CHECK_LE(model.StartTime(), start_time);
            CHECK_LE(end_time, model.EndTime());

            actions_body.push_back({
                                           {"station",    stations[action.Station()]},
                                           {"key_rate",   key_rate},
                                           {"start_time", start_time},
                                           {"end_time",   end_time}
                                   });
        }

        return {
                {"total_key_rate", solution.TotalKeyRate()},
                {"stations",       stations_body},
                {"actions",        actions_body}
        };
    }

    template<typename StreamType>
    nlohmann::json CpSolutionJsonWriter<StreamType>::ToJson(const InferredModel &model) const {
        nlohmann::json body{
                {"start_time",      model.StartTime()},
                {"end_time",        model.EndTime()},
                {"stations",        model.Stations()},
                {"dummy_station",   model.DummyStation()},
                {"step_duration",   model.StepDuration()},
                {"switch_duration", model.SwitchDuration()}
        };

        if (model.StationKeyRate().HasUpperBound() || model.StationKeyRate().HasLowerBound()) {
            nlohmann::json station_key_rate_document;

            if (model.StationKeyRate().HasLowerBound()) {
                station_key_rate_document["lower_bound"] = model.StationKeyRate().LowerBound();
            }

            if (model.StationKeyRate().HasUpperBound()) {
                station_key_rate_document["upper_bound"] = model.StationKeyRate().UpperBound();
            }

            body["station_key_rate"] = station_key_rate_document;
        }

        if (model.TotalKeyRate().HasUpperBound() || model.TotalKeyRate().HasLowerBound()) {
            nlohmann::json total_key_rate;

            if (model.StationKeyRate().HasLowerBound()) {
                total_key_rate["lower_bound"] = model.TotalKeyRate().LowerBound();
            }

            if (model.StationKeyRate().HasUpperBound()) {
                total_key_rate["upper_bound"] = model.TotalKeyRate().UpperBound();
            }

            body["total_key_rate"] = total_key_rate;
        }

        return body;
    }

    template<typename StreamType>
    void CpSolutionJsonWriter<StreamType>::Write(const InferredModel &model, const std::vector<CpSolution> &solutions) {
        nlohmann::json body;

        body["metadata"] = ToJson(model);

        std::vector<nlohmann::json> solutions_body;
        for (const auto &solution : solutions) {
            solutions_body.push_back(ToJson(model, solution));
        }
        body["solutions"] = solutions_body;

        stream_ << std::setw(4) << body << std::endl;
    }

    template<typename StreamType>
    void CpSolutionJsonWriter<StreamType>::Write(const InferredModel &model, const CpSolution &solution) {
        Write(model, std::vector<CpSolution>{solution});
    }
}

#endif //QUAKE_CP_SOLUTION_JSON_WRITER_H
