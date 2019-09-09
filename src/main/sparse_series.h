#ifndef QUAKE_SPARSE_SERIES_H
#define QUAKE_SPARSE_SERIES_H

namespace quake {

    template<typename ValueType>
    class SparseSeries {
    public:
        SparseSeries(const SparseSeries<ValueType> &other)
                : update_frequency_{other.update_frequency_},
                  period_{other.period_},
                  values_{other.values_} {
            CHECK(CheckConsistency());
        }

        SparseSeries(SparseSeries<ValueType> &&other) noexcept
                : update_frequency_{std::move(other.update_frequency_)},
                  period_{std::move(other.period_)},
                  values_{std::move(other.values_)} {
            CHECK(CheckConsistency());
        }

        SparseSeries(boost::posix_time::time_duration update_frequency,
                     boost::posix_time::time_period period,
                     std::vector<ValueType> values)
                : update_frequency_{std::move(update_frequency)},
                  period_{period},
                  values_{std::move(values)} {
            CHECK(CheckConsistency());
        }

        inline const std::vector<ValueType> &Values() const { return values_; }

        inline const boost::posix_time::time_period &Period() const { return period_; }

        inline const boost::posix_time::time_duration &UpdateFrequency() const { return update_frequency_; }

        ValueType operator()(const boost::posix_time::ptime &value) const {
            if (!period_.contains(value) && period_.end() != value) {
                LOG(WARNING) << "Time " << value << " is outside the period of observations " << period_ << ".";
                return 0;
            }

            const auto time_from_start = value - period_.begin();
            CHECK(!time_from_start.is_negative());

            const auto left_index = static_cast<std::size_t>(time_from_start.total_seconds() / update_frequency_.total_seconds());

            // old behaviour
//                const auto remaining_seconds = time_from_start.total_seconds() - left_index * update_frequency_.total_seconds();
//                if (remaining_seconds > update_frequency_.total_seconds() / 2) {
//                    const auto right_index = left_index + 1;
//                    if (right_index < values_.size()) {
//                        LOG(INFO) << "Right index: " << right_index;
//                        return values_.at(right_index);
//                    }
//                }

            return values_.at(left_index);
        }

        SparseSeries<ValueType> Trim(const boost::posix_time::time_period &period) const {
            CHECK(period_.contains(period)) << period << " must be contained in " << period_;

            const auto update_freq_seconds = update_frequency_.total_seconds();
            const auto begin_offset_seconds = (period_.begin() - period.begin()).total_seconds();
            CHECK(begin_offset_seconds % update_freq_seconds == 0);
            CHECK(period.length().total_seconds() % update_freq_seconds == 0);

            const auto begin_offset = static_cast<std::size_t>(begin_offset_seconds / update_freq_seconds);
            const auto length = static_cast<std::size_t>(period.length().total_seconds() / update_freq_seconds);

            std::vector<ValueType> trimmed_values{values_.begin() + begin_offset, values_.begin() + begin_offset + length};
            return {update_frequency_, period, std::move(trimmed_values)};
        }

    private:
        bool CheckConsistency() const {
            const auto update_freq_seconds = update_frequency_.total_seconds();
            if (period_.begin().time_of_day().total_seconds() % update_freq_seconds != 0) {
                return false; // begin period is not aligned
            }

            if (period_.end().time_of_day().total_seconds() % update_freq_seconds != 0) {
                return false; // end period is not aligned
            }

            const auto num_pos = period_.length().total_seconds() / update_freq_seconds;
            return values_.size() == num_pos; // size matches the expected length
        }

        boost::posix_time::time_duration update_frequency_;
        boost::posix_time::time_period period_;
        std::vector<ValueType> values_;
    };
}
#endif //QUAKE_SPARSE_SERIES_H
