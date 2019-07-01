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

#ifndef QUAKE_CP_SOLUTION_H
#define QUAKE_CP_SOLUTION_H

#include <vector>

#include <boost/config.hpp>
#include <boost/filesystem.hpp>

#include <nlohmann/json.hpp>

#include <ortools/base/basictypes.h>

namespace quake {

    class CpSolution {
    public:
        class Job {
        public:
            Job() = default;

            Job(int64 start, int64 duration, int64 station, int64 key_rate);

            inline int64 Start() const { return start_; }

            inline int64 Duration() const { return duration_; }

            inline int64 Station() const { return station_; }

            inline int64 KeysTransferred() const { return keys_transferred_; }

            friend void to_json(nlohmann::json &json, const Job &value);

            friend void from_json(const nlohmann::json &json, Job &value);

        private:
            int64 start_;
            int64 duration_;
            int64 station_;
            int64 keys_transferred_;
        };

        CpSolution();

        explicit CpSolution(std::vector<Job> actions);

        CpSolution(std::vector<Job> actions, std::vector<int64> final_buffer);

        static CpSolution Load(const boost::filesystem::path &path);

        inline int64 TotalKeyRate() const { return total_key_rate_; }

        inline int64 MaxStationKeyRate() const { return max_station_key_rate_; }

        inline int64 MinStationKeyRate() const { return min_station_key_rate_; }

        inline int64 TransferredKeys(std::size_t index) const {
            if (index < transferred_keys_.size()) {
                return transferred_keys_[index];
            }
            return 0;
        }

        inline int64 FinalBuffer(std::size_t index) const {
            if (index < final_buffer_.size()) {
                return final_buffer_[index];
            }
            return 0;
        }

        inline int64 MinTransferSize() const {
            int64 min_transfer_size = kint64max;
            for (const auto &job : actions_) {
                min_transfer_size = std::min(job.KeysTransferred(), min_transfer_size);
            }
            return min_transfer_size;
        }

        inline const std::vector<Job> &Jobs() const { return actions_; }

        inline const std::vector<int64> &TransferredKeys() const { return transferred_keys_; }

        inline const std::vector<int64> &FinalBuffer() const { return final_buffer_; }

        friend void to_json(nlohmann::json &json, const CpSolution &value);

        friend void from_json(const nlohmann::json &json, CpSolution &value);

    private:
        std::vector<Job> actions_;

        int64 total_key_rate_;
        int64 max_station_key_rate_;
        int64 min_station_key_rate_;

        std::vector<int64> transferred_keys_;
        std::vector<int64> final_buffer_;
    };

    void to_json(nlohmann::json &json, const CpSolution &value);

    void from_json(const nlohmann::json &json, CpSolution &value);

    void from_json(const nlohmann::json &json, CpSolution::Job &value);

    void to_json(nlohmann::json &json, const CpSolution::Job &value);
}

std::ostream &operator<<(std::ostream &out, const quake::CpSolution::Job &job);

#endif //QUAKE_CP_SOLUTION_H
