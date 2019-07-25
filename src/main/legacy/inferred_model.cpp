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

#include "inferred_model.h"

#include <algorithm>
#include <numeric>

#include <boost/format.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <glog/logging.h>

#include "util/error.h"
#include "util/hash.h"
#include "util/resources.h"
#include "legacy/cp_solution.h"
#include "minizinc_reader.h"
#include "ground_station.h"
#include "forecast.h"
#include "sunset_sunrise_database.h"
#include "sunset_sunrise_reader.h"

quake::InferredModel::InferredModel(quake::MiniZincDataModel<int64, int64> data_model)
        : data_model_(std::move(data_model)),
          total_key_rate_{0, std::numeric_limits<int64>::max()},
          key_rate_cumulative_{0, std::numeric_limits<int64>::max()},
          station_key_rate_{0, std::numeric_limits<int64>::max()} {
    CHECK_LE(data_model_.Stations().size(), std::numeric_limits<int>::max());

    for (const auto &station_key_rate : data_model_.KeyRate()) {
        for (const auto &value : station_key_rate) {
            key_rate_1d_.push_back(static_cast<int64>(round(value)));
        }
    }

    station_index_multiplier_ = TimeRange();
    cumulative_station_index_multiplier_ = station_index_multiplier_ + 1;
    for (const auto &station_key_rate_cumulative : data_model_.KeyRateCumulative()) {
        for (const auto &value : station_key_rate_cumulative) {
            key_rate_cumulative_1d_.push_back(static_cast<int64>(round(value)));
        }
    }
    CHECK_EQ(cumulative_station_index_multiplier_ * data_model_.Stations().size(), key_rate_cumulative_1d_.size());
    weather_adjusted_key_rate_1d_ = key_rate_1d_;
    weather_adjusted_key_rate_cumulative_1d_ = key_rate_cumulative_1d_;

    reconfiguration_time_.resize(static_cast<std::size_t>(cumulative_station_index_multiplier_), 0);
    for (auto time_index = 1; time_index < cumulative_station_index_multiplier_; ++time_index) {
        reconfiguration_time_[time_index] = std::max(
                data_model_.SwitchDuration()
                - (data_model_.TimeOffset()[time_index]
                   - data_model_.TimeOffset()[time_index - 1]
                   - data_model_.StepDuration()),
                static_cast<int64>(0));
    }

    initial_buffer_override_ = data_model_.InitialBuffer();
    key_consumption_override_ = data_model_.KeyConsumption();

#if DEBUG
    for (auto station_index = 0; station_index < StationCount(); ++station_index) {
        for (auto time_index = 1; time_index < cumulative_station_index_multiplier_; ++time_index) {
            CHECK_LE(key_rate_cumulative_1d_.at(CumulativeIndex(station_index, time_index - 1)),
                     key_rate_cumulative_1d_.at(CumulativeIndex(station_index, time_index)))
                << boost::format("key_rate_cumulative[%1%] < key_rate_cumulative[%2%]")
                   % CumulativeIndex(station_index, time_index - 1)
                   % CumulativeIndex(station_index, time_index);
        }
    }

    for (auto station_index = 0; station_index < data_model_.Stations().size(); ++station_index) {
        CHECK_EQ(data_model_.KeyRateCumulative().at(station_index).size(), TimeRange() + 1);
    }

    for (auto time_index = 0; time_index < cumulative_station_index_multiplier_; ++time_index) {
        CHECK_GE(reconfiguration_time_[time_index], 0);
    }
#endif

    const auto min_max_element = std::minmax_element(std::cbegin(key_rate_cumulative_1d_),
                                                     std::cend(key_rate_cumulative_1d_));

    key_rate_cumulative_.SetLowerBound(*min_max_element.first);
    key_rate_cumulative_.SetUpperBound(*min_max_element.second);
}

quake::InferredModel quake::InferredModel::Load(const boost::filesystem::path &path) {
    std::ifstream input_stream;

    VLOG(1) << "Loading model from " << path;
    input_stream.open(path.string());
    if (input_stream.bad()) {
        throw util::OnFailedReadInput(path);
    }

    MiniZincReader<std::ifstream, int64> reader{std::move(input_stream)};
    auto raw_data_model = reader.Read();
    reader.Close();

    MiniZincDataModel<int64, int64> data_model{std::move(raw_data_model)};

    VLOG(1) << "Model loaded from " << path;
    return quake::InferredModel{std::move(data_model)};
}

boost::posix_time::ptime quake::InferredModel::Time(int64 start_index) const {
    if (start_index == data_model_.Time().size()) {
        return data_model_.EndTime();
    }

    const auto &time_offset = data_model_.TimeOffset();
    const auto relative_offset = time_offset.at(start_index);
    CHECK_GE(relative_offset, 0);
    return StartTime() + StepsToDuration(relative_offset);
}

boost::posix_time::ptime quake::InferredModel::EndTime(int64 start_index, int64 steps) const {
    const auto end_time = Time(start_index + steps);

#if DEBUG
    const auto start_time = Time(start_index);
    auto leap_duration = boost::posix_time::seconds(0);
    auto last_time = start_time;
    for (auto current_step = 1; current_step <= steps; ++current_step) {
        auto current_time = Time(start_index + current_step);
        auto leap_increase = current_time - last_time - boost::posix_time::seconds(1);
        CHECK(!leap_increase.is_negative());
        leap_duration += leap_increase;
        last_time = current_time;
    }
    CHECK_EQ(start_time + leap_duration + StepsToDuration(steps), end_time);
    CHECK_GE(end_time, start_time);
    CHECK_LE(end_time, EndTime());
#endif

    return end_time;
}

void quake::InferredModel::CheckConsistency(const quake::CpSolution &solution) const {
    const auto &jobs = solution.Jobs();
    if (jobs.empty()) {
        return;
    }

    // check individual jobs
    const auto jobs_size = jobs.size();
    for (auto job_index = 0; job_index < jobs_size; ++job_index) {
        const auto &job = jobs[job_index];
        CheckConsistency(job);
    }

    // check jobs do not overlap and do not have gaps
    // check begin and end transfer indices are consistent
    CHECK_EQ(GetStartTransferTimeIndex(jobs[0]), 0);
    for (auto job_index = 1; job_index < jobs_size; ++job_index) {
        CHECK_LE(GetEndTransferIndex(jobs[job_index - 1]), jobs[job_index].Start());
    }
    CHECK_EQ(GetEndTransferIndex(jobs[jobs_size - 1]), TimeRange());
}

int64 quake::InferredModel::GetStartTransferTimeIndex(const quake::CpSolution::Job &job) const {
    return GetStartTransferTimeIndex(job.Station(), job.Start());
}

int64 quake::InferredModel::GetStartTransferTimeIndex(int64 station, int64 time) const {
    if (time == 0) { return 0; }
    return time + (station != DummyStationIndex()) * ReconfigurationTime()[time];
}

int64 quake::InferredModel::GetEndTransferIndex(const quake::CpSolution::Job &job) const {
    return job.Start() + job.Duration();
}

quake::SunsetSunriseDatabase LoadSunsetSunriseDatabase(const std::vector<std::string> &stations) {
    std::vector<quake::GroundStation> ground_stations;
    for (const auto &station : stations) {
        const auto ground_station = quake::GroundStation::FromNameOrNone(station);
        if (ground_station == quake::GroundStation::None) {
            continue;
        }
        ground_stations.push_back(ground_station);
    }

    quake::SunsetSunriseDatabase sunset_sunrise_database;
    sunset_sunrise_database.Load(ground_stations);

    return sunset_sunrise_database;
}

std::vector<quake::InferredModel::StageRecord> quake::InferredModel::InferStages() const {
    CHECK(!this->Stations().empty());

    const auto sunset_sunrise_data = LoadSunsetSunriseDatabase(this->Stations());

    const auto start_time = StartTime();
    const auto start_date = start_time.date();
    const auto end_date = EndTime().date();

    auto offset_index = 0;
    const std::vector<int64> &time_offsets = TimeOffset();
    const auto offset_size = time_offsets.size();

    std::vector<StageRecord> stages;
    auto skipped_stages = 0;
    stages.emplace_back(StartTime() + boost::posix_time::seconds(time_offsets[0]), 0, kint64max);
    for (auto current_date = start_date + boost::gregorian::days(1);
         current_date <= end_date + boost::gregorian::days(1);
         current_date += boost::gregorian::days(1)) {

        const auto sunset_time = sunset_sunrise_data.MinSunset(current_date);
        for (auto offset_length = 0; offset_index < offset_size; ++offset_index, ++offset_length) {
            const auto current_offset = time_offsets[offset_index];
            const auto current_time = start_time + boost::posix_time::seconds(current_offset);
            if (current_time >= sunset_time) {
                if (offset_length > 3) { // ignore stages of duration less than 3 seconds
                    stages.emplace_back(current_time, offset_index, 0);
                } else {
                    ++skipped_stages;
                }
                break;
            }
        }
    }

    if (stages.size() > 1) {
        std::vector<int> stage_sizes;
        boost::accumulators::accumulator_set<int,
                boost::accumulators::features<boost::accumulators::tag::mean, boost::accumulators::tag::variance>
        > stage_size_accumulator;
        for (auto position = 1; position < stages.size(); ++position) {
            const auto offset = stages[position].OffsetIndex - stages[position - 1].OffsetIndex;
            stage_size_accumulator(offset);
            stage_sizes.push_back(offset);
        }
        const auto last_offset = offset_size - stages[stages.size() - 1].OffsetIndex;
        stage_size_accumulator(last_offset);
        stage_sizes.push_back(last_offset);

        const auto mean_stage_size = boost::accumulators::mean(stage_size_accumulator);
        const auto std_stage_size = sqrt(boost::accumulators::variance(stage_size_accumulator));
        for (auto position = 0; position < stage_sizes.size(); ++position) {
            const auto absolute_error = abs(mean_stage_size - stage_sizes[position]);
            if (absolute_error >= 3 * std_stage_size) {
                std::ostringstream msg;
                msg << "Size of the stage is too different when compared to others: [ ";
                msg << stage_sizes[0];
                for (auto position = 1; position < stage_sizes.size(); ++position) {
                    msg << ", " << stage_sizes[position];
                }
                msg << "]";

                LOG(FATAL) << msg.str();
            }
        }
    }

    const auto time_period = TimePeriod();
    auto days = (time_period.end().date() - time_period.begin().date()).days();
    if (time_period.end().time_of_day() >= boost::posix_time::hours(12)) {
        ++days;
    }

    CHECK_EQ(days, stages.size() + skipped_stages);

    // TODO2 test - are there any time slots after last sunrise over ground station?
    // TODO2 test - are there any positive transfer after last sunrise over ground station
    return stages;
}

quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback
quake::InferredModel::GetWeatherAdjustedCumulativeKeyRate(int64 ground_station) const {
    return WeatherAdjustedCumulativeKeyRateCallback(ground_station, this);
}

static const auto MAX_CLOUD_COVER = 100;

int64 quake::InferredModel::WeatherAdjustedKeyRate(int64 time_index, int64 station_index, int64 cloud_cover) const {
    CHECK_GE(cloud_cover, 0);
    CHECK_LE(cloud_cover, MAX_CLOUD_COVER);

    const auto key_rate_index = Index(station_index, time_index);
    return (MAX_CLOUD_COVER - cloud_cover) * key_rate_1d_[key_rate_index];
}

void quake::InferredModel::Apply(const quake::Forecast &forecast) {
    for (auto station_index = 0; station_index < StationCount(); ++station_index) {
        const auto &ground_station = Station(station_index);

        if (ground_station == GroundStation::None) { continue; }

        for (auto time_index = 0; time_index < cumulative_station_index_multiplier_; ++time_index) {
            const auto current_time = Time(time_index);
            const auto cloud_cover = forecast.GetCloudCover(ground_station, current_time);

            const auto key_rate_index = Index(station_index, time_index);
            weather_adjusted_key_rate_1d_[key_rate_index]
                    = WeatherAdjustedKeyRate(time_index, station_index, cloud_cover);
        }
    }

    std::vector<int64> candidate_cumulative_key_rate_1d(weather_adjusted_key_rate_cumulative_1d_.size(), 0);
    for (auto station_index = 1; station_index < StationCount(); ++station_index) {
        for (auto time_index = 1; time_index < cumulative_station_index_multiplier_; ++time_index) {
            candidate_cumulative_key_rate_1d[CumulativeIndex(station_index, time_index)]
                    = candidate_cumulative_key_rate_1d[CumulativeIndex(station_index, time_index - 1)] +
                      weather_adjusted_key_rate_1d_[Index(station_index, time_index - 1)];
        }
    }

    weather_adjusted_key_rate_cumulative_1d_ = candidate_cumulative_key_rate_1d;
#if DEBUG
    for (auto station_index = 1; station_index < StationCount(); ++station_index) {
        for (auto time_index = 1; time_index < cumulative_station_index_multiplier_; ++time_index) {
            CHECK_LE(weather_adjusted_key_rate_cumulative_1d_[CumulativeIndex(station_index, time_index - 1)],
                     weather_adjusted_key_rate_cumulative_1d_[CumulativeIndex(station_index, time_index)]);
        }
    }
#endif

    if (!weather_adjusted_) {
        std::vector<int64> initial_buffer_override;
        for (const auto value : initial_buffer_override_) {
            initial_buffer_override.push_back(value * MAX_CLOUD_COVER);
        }
        initial_buffer_override_ = initial_buffer_override;

        std::vector<int64> key_consumption_override;
        for (const auto value : key_consumption_override_) {
            key_consumption_override.push_back(value * MAX_CLOUD_COVER);
        }
        key_consumption_override_ = key_consumption_override;
    }
    weather_adjusted_ = true;
}

struct AbsoluteError {
    double operator()(double actual_value, double expected_value) const {
        return abs(actual_value - expected_value);
    }
};

struct RelativeError {
    double operator()(double actual_value, double expected_value) const {
        return abs((actual_value - expected_value) / expected_value);
    }
};

std::vector<std::pair<quake::GroundStation, double> >
quake::InferredModel::GetAbsoluteError(const quake::CpSolution &solution) const {
    return GetError<AbsoluteError>(solution);
}

std::vector<std::pair<quake::GroundStation, double> >
quake::InferredModel::GetRelativeError(const quake::CpSolution &solution) const {
    return GetError<RelativeError>(solution);
}

int64 quake::InferredModel::GetMinRateScalingFactor() const {
    static const int64 MAX_RATE_SCALING_FACTOR = 10000;
    static const double MIN_CAST_PRECISION = 0.0001;

    const auto &transfer_share = TransferShare();
    const auto transfer_share_size = transfer_share.size();

    int64 rate_scaling_factor = 1;
    while (rate_scaling_factor < MAX_RATE_SCALING_FACTOR) {
        int64 station_index;
        for (station_index = 1; station_index < transfer_share_size; ++station_index) {
            const auto scaled_transfer_share = transfer_share[station_index] * rate_scaling_factor;
            const auto cast_error = scaled_transfer_share - static_cast<int64>(scaled_transfer_share);
            if (cast_error > MIN_CAST_PRECISION) { break; }
        }

        if (station_index >= transfer_share_size) {
            return rate_scaling_factor;
        }

        rate_scaling_factor *= 10;
    }

    return rate_scaling_factor;
}

std::unordered_map<
        boost::gregorian::date,
        std::pair<boost::posix_time::ptime, boost::posix_time::ptime>
> quake::InferredModel::LoadSunsetSunrise(quake::GroundStation ground_station) const {
    quake::util::Resources resources{"~/dev/quake/data"};
    quake::SunsetSunriseReader sunset_sunrise_reader{quake::util::Resources::DEFAULT_LOCAL_TIME_ZONE};
    auto sunset_sunrise_pairs = sunset_sunrise_reader.Read(resources.SunsetSunriseData(ground_station));
    DCHECK_EQ(ground_station, sunset_sunrise_pairs.first);

    std::unordered_map<boost::gregorian::date,
            std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > sunset_sunrise_row;
    for (const auto &sunset_sunrise_pair : sunset_sunrise_pairs.second) {
        DCHECK_EQ(sunset_sunrise_pair.first.date(), sunset_sunrise_pair.second.date());
        sunset_sunrise_row.emplace(sunset_sunrise_pair.first.date(), sunset_sunrise_pair);
    }

    return sunset_sunrise_row;
}

void quake::InferredModel::Apply(const quake::Solution &previous_solution) {
    std::vector<int64> initial_buffer{0};
    std::vector<double> key_consumption_ratio;

    const auto &station_names = Stations();
    for (auto station_index = 1; station_index < StationCount(); ++station_index) {
        const auto &station_name = station_names[station_index];
        const auto station = GroundStation::FromName(station_name);
        const auto final_buffer_it = previous_solution.FinalBuffers().find(station);
        CHECK(final_buffer_it != std::cend(previous_solution.FinalBuffers()));

        auto final_buffer_value = final_buffer_it->second;
        if (weather_adjusted_) {
            final_buffer_value *= MAX_CLOUD_COVER;
        }

        initial_buffer.push_back(final_buffer_value);
        key_consumption_ratio.push_back(static_cast<double>(final_buffer_value) / TransferShare(station_index));
    }

    const auto key_consumption_ratio_it
            = std::min_element(std::cbegin(key_consumption_ratio), std::cend(key_consumption_ratio));
    const auto min_key_consumption_ratio = *key_consumption_ratio_it;
    const auto stages = InferStages();
    const auto days = stages.size();

    std::vector<int64> key_consumption{0};
    for (auto station_index = 1; station_index < StationCount(); ++station_index) {
        const auto station_daily_key_consumption
                = min_key_consumption_ratio * TransferShare(station_index) / days;
        key_consumption.push_back(station_daily_key_consumption);
    }

    CHECK_EQ(key_consumption_override_.size(), key_consumption.size());
    CHECK_EQ(initial_buffer_override_.size(), initial_buffer.size());
    for (auto station_index = 0; station_index < StationCount(); ++station_index) {
        CHECK_GE(initial_buffer[station_index] - days * key_consumption[station_index], 0);
    }

    initial_buffer_override_ = initial_buffer;
    key_consumption_override_ = key_consumption;
}

int64 quake::InferredModel::TransferredKeys(const quake::GroundStation &station,
                                            const boost::posix_time::ptime &start_time,
                                            const boost::posix_time::ptime &end_time) const {
    const auto station_index = GetStationIndex(station);
    const auto start_index = GetStartIndex(start_time);
    const auto end_index = GetEndIndex(end_time);

    auto total_keys = 0;
    for (auto time_index = start_index; time_index < end_index; ++time_index) {
        const auto key_rate_index = this->Index(station_index, time_index);
        const auto key_rate = this->KeyRate1d().at(key_rate_index);
        total_keys += key_rate;
    }

    const auto &key_rate_cumulative = this->KeyRateCumulative1d();
    const auto start_cumul_index = this->CumulativeIndex(station_index, start_index);
    const auto end_cumul_index = this->CumulativeIndex(station_index, end_index);
    const auto total_cumul_keys = key_rate_cumulative.at(end_cumul_index) - key_rate_cumulative.at(start_cumul_index);
    CHECK_EQ(total_cumul_keys, total_keys);

    return total_keys;
}

int64 quake::InferredModel::WeatherAdjustedTransferredKeys(const quake::GroundStation &station,
                                                           const boost::posix_time::ptime &start_time,
                                                           const boost::posix_time::ptime &end_time,
                                                           const Forecast &forecast) const {
    const auto station_index = GetStationIndex(station);
    const auto start_index = GetStartIndex(start_time);
    const auto end_index = GetEndIndex(end_time);

    auto total_keys = 0;
    for (auto time_index = start_index; time_index < end_index; ++time_index) {
        const auto current_time = Time(time_index);
        const auto cloud_cover = forecast.GetCloudCover(station, current_time);
        total_keys += WeatherAdjustedKeyRate(time_index, station_index, cloud_cover);
    }
    return total_keys;
}

int64 quake::InferredModel::WeatherAdjustedTransferredKeys(const quake::GroundStation &station,
                                                           const boost::posix_time::ptime &start_time,
                                                           const boost::posix_time::ptime &end_time) const {
    const auto station_index = GetStationIndex(station);
    const auto end_index = GetEndIndex(end_time);
    const auto start_index = GetStartIndex(start_time);

    const auto &time_offset = this->TimeOffset();
    auto total_keys = 0;
    for (auto time_index = start_index; time_index < end_index; ++time_index) {
        const auto key_rate_index = this->Index(station_index, time_index);
        const auto key_rate = this->weather_adjusted_key_rate_1d_.at(key_rate_index);
        total_keys += key_rate;
    }

    const auto &weather_adjusted_key_rate_cumulative = this->weather_adjusted_key_rate_cumulative_1d_;
    const auto start_cumul_index = this->CumulativeIndex(station_index, start_index);
    const auto end_cumul_index = this->CumulativeIndex(station_index, end_index);
    const auto total_cumul_keys = weather_adjusted_key_rate_cumulative.at(end_cumul_index)
                                  - weather_adjusted_key_rate_cumulative.at(start_cumul_index);
    CHECK_EQ(total_cumul_keys, total_keys);

    return total_keys;
}


quake::Solution quake::InferredModel::Create(std::unordered_map<quake::GroundStation,
        std::vector<boost::posix_time::time_period> > observations) const {
    std::unordered_map<GroundStation, int64> keys_transferred;
    for (auto station_index = 0; station_index < this->StationCount(); ++station_index) {
        keys_transferred[Station(station_index)] = 0;
    }

    for (const auto &station_observations : observations) {
        int64 stations_key_transferred = 0;

        if (this->IsWeatherAdjusted()) {
            for (const auto &observation: station_observations.second) {
                stations_key_transferred
                        += this->WeatherAdjustedTransferredKeys(station_observations.first,
                                                                observation.begin(),
                                                                observation.end());
            }
        } else {
            for (const auto &observation: station_observations.second) {
                stations_key_transferred
                        += this->TransferredKeys(station_observations.first, observation.begin(), observation.end());
            }
        }

        keys_transferred[station_observations.first] = stations_key_transferred;
    }

    const auto days_length = this->InferStages().size();
    std::unordered_map<GroundStation, int64> station_final_buffer;
    for (auto station_index = 1; station_index < this->StationCount(); ++station_index) {
        const auto station = this->Station(station_index);
        station_final_buffer[station] = this->InitialBuffer(station_index)
                                        + keys_transferred.at(station)
                                        - days_length * this->KeyConsumption(station_index);
    }

    if (this->IsWeatherAdjusted()) {
        for (auto &station_buffer_pair : station_final_buffer) {
            station_buffer_pair.second /= 100;
        }
    }

    return {{}, std::move(observations), std::move(station_final_buffer)};
}

int64 quake::InferredModel::GetStationIndex(const quake::GroundStation &station) const {
    int64 current_index = 0;
    for (const auto &station_name : this->Stations()) {
        if (station_name == station.name()) {
            return current_index;
        }

        ++current_index;
    }

    LOG(FATAL) << "Failed to find an index for the station " << station;
    return 0;
}

int64 quake::InferredModel::GetStartIndex(const boost::posix_time::ptime &time) const {
    int64 start_offset = (time - this->StartTime()).total_seconds();
    CHECK_GE(start_offset, 0);

    // start offset could be before start of simulations because it is rounded to full minutes

    auto start_index = 1;
    const auto &time_offset = this->TimeOffset();
    while (start_index < time_offset.size() && time_offset[start_index] <= start_offset) {
        ++start_index;
    }
    --start_index;
    CHECK_LE(start_offset, time_offset[start_index]);
    return start_index;
}

int64 quake::InferredModel::GetEndIndex(const boost::posix_time::ptime &time) const {
    int64 end_offset = (time - this->StartTime()).total_seconds();
    CHECK_GE(end_offset, 0);

    auto end_index = 1;
    const auto &time_offset = this->TimeOffset();
    while (end_index < time_offset.size() && time_offset[end_index] < end_offset) {
        ++end_index;
    }

    if (end_offset > time_offset[end_index]) {
        CHECK_EQ(end_index, time_offset.size());
    }
    return end_index;
}

void quake::InferredModel::CheckConsistency(const quake::CpSolution::Job &job) const {
    const auto start_transfer_time = GetStartTransferTimeIndex(job);
    const auto end_transfer_time = GetEndTransferIndex(job);

    CHECK_EQ(job.Start() + job.Duration(), end_transfer_time);
    if (job.Station() == DummyStationIndex()) {
        CHECK_EQ(start_transfer_time, end_transfer_time);
        CHECK_EQ(job.Duration(), 0);
        CHECK_EQ(job.KeysTransferred(), 0);
    } else {
        CHECK_LE(start_transfer_time, end_transfer_time);
        auto key_rate = 0;
        for (auto index = start_transfer_time; index < end_transfer_time; ++index) {
            key_rate += WeatherAdjustedKeyRate(job.Station(), index);
        }

        CHECK_GT(job.Duration(), 0);
        CHECK_GT(job.KeysTransferred(), 0);
        CHECK_EQ(job.KeysTransferred(), key_rate)
            << Station(job.Station()) << " "
            << Time(job.Start()) << " " << Time(job.Start() + job.Duration());
    }
}

quake::InferredModel::StageRecord::StageRecord(boost::posix_time::ptime start_time,
                                               std::size_t offset_index,
                                               int64 marginal_change)
        : StartTime(start_time),
          OffsetIndex(offset_index),
          MarginalChange(marginal_change) {}

quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback::WeatherAdjustedCumulativeKeyRateCallback(
        int ground_station, const quake::InferredModel *model)
        : ground_station_{ground_station},
          model_{model} {}

int64 quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback::operator()(int64 time_index) const {
    return model_->WeatherAdjustedKeyCumulativeRate(ground_station_, time_index);
}

quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback::WeatherAdjustedCumulativeKeyRateCallback(
        const quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback &other)
        : WeatherAdjustedCumulativeKeyRateCallback(other.ground_station_, other.model_) {}

quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback &
quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback::operator=(
        const quake::InferredModel::WeatherAdjustedCumulativeKeyRateCallback &other) {
    ground_station_ = other.ground_station_;
    model_ = other.model_;
    return *this;
}
