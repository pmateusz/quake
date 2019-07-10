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

#ifndef QUAKE_HASH_H
#define QUAKE_HASH_H

#include <boost/config.hpp>
#include <boost/functional/hash.hpp>
#include <boost/date_time.hpp>

namespace std {

    template<>
    struct hash<boost::gregorian::date> {
        std::size_t operator()(const boost::gregorian::date &date) const noexcept {
            std::size_t seed = 0;
            boost::hash_combine(seed, date.day().as_number());
            boost::hash_combine(seed, date.month().as_number());
            boost::hash_combine(seed, static_cast<unsigned short>(date.year()));
            return seed;
        }
    };

    template<>
    struct hash<boost::posix_time::time_duration> {
        std::size_t operator()(const boost::posix_time::time_duration &time_duration) const noexcept {
            return time_duration.total_nanoseconds();
        }
    };

    template<>
    struct hash<boost::posix_time::ptime> {
        std::size_t operator()(const boost::posix_time::ptime &datetime) const noexcept {
            static const std::hash<boost::gregorian::date> date_hasher;
            static const std::hash<boost::posix_time::time_duration> time_duration_hasher;

            std::size_t seed = 0;
            boost::hash_combine(seed, date_hasher(datetime.date()));
            boost::hash_combine(seed, time_duration_hasher(datetime.time_of_day()));
            return seed;
        }
    };

    template<>
    struct hash<boost::posix_time::time_period> {
        std::size_t operator()(const boost::posix_time::time_period &time_period) const noexcept {
            static const std::hash<boost::posix_time::ptime> ptime_hasher;

            std::size_t seed = 0;
            boost::hash_combine(seed, ptime_hasher(time_period.begin()));
            boost::hash_combine(seed, ptime_hasher(time_period.end()));
            return seed;
        }
    };
}
#endif //QUAKE_HASH_H
