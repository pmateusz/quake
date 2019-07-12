#ifndef QUAKE_INTERVAL_STATION_H
#define QUAKE_INTERVAL_STATION_H

#include <boost/config.hpp>
#include <boost/date_time.hpp>

#include "util/hash.h"

namespace quake {

    class IntervalStation {
    public:
        IntervalStation(std::size_t station_index, boost::posix_time::time_period period)
                : station_index_{station_index},
                  period_{period} {}

        inline bool operator==(const IntervalStation &other) const {
            return station_index_ == other.station_index_ && period_ == other.period_;
        }

        inline std::size_t StationIndex() const { return station_index_; }

        inline const boost::posix_time::time_period &Period() const { return period_; }

    private:
        std::size_t station_index_;
        boost::posix_time::time_period period_;
    };
}

namespace std {

    template<>
    struct hash<quake::IntervalStation> {
        std::size_t operator()(const quake::IntervalStation &interval) const {
            static const std::hash<boost::posix_time::time_period> period_hasher;

            std::size_t seed = 0;
            boost::hash_combine(seed, boost::hash_value(interval.StationIndex()));
            boost::hash_combine(seed, period_hasher(interval.Period()));
            return seed;
        }
    };
}

#endif //QUAKE_INTERVAL_STATION_H
