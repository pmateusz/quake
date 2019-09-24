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

#ifndef QUAKE_GROUND_STATION_H
#define QUAKE_GROUND_STATION_H

#include <functional>
#include <ostream>
#include <string>

#include <boost/functional/hash.hpp>

#include <pykep/third_party/libsgp4/CoordGeodetic.h>

#include <nlohmann/json.hpp>

namespace quake {

    class GroundStation {
    public:
        static const GroundStation London;
        static const GroundStation Glasgow;
        static const GroundStation Thurso;
        static const GroundStation Manchester;
        static const GroundStation Birmingham;
        static const GroundStation Bristol;
        static const GroundStation Ipswich;
        static const GroundStation Cambridge;
        static const GroundStation York;
        static const GroundStation Belfast;
        static const GroundStation None;

        static const std::vector<GroundStation> All;

        GroundStation();

        GroundStation(const CoordGeodetic &coordinates, std::string name);

        GroundStation(const GroundStation &other);

        GroundStation(GroundStation &&other) noexcept;

        GroundStation &operator=(const GroundStation &other);

        GroundStation &operator=(GroundStation &&other) noexcept;

        static const GroundStation &FromNameOrNone(const std::string &name);

        static const GroundStation &FromName(const std::string &name);

        inline bool operator==(const GroundStation &other) const noexcept {
            return coordinates_ == other.coordinates_
                   && name_ == other.name_;
        }

        inline bool operator!=(const GroundStation &other) const noexcept {
            return !this->operator==(other);
        }

        inline std::size_t hash() const noexcept {
            std::size_t seed = 0;
            boost::hash_combine(seed, coordinates_.latitude);
            boost::hash_combine(seed, coordinates_.longitude);
            boost::hash_combine(seed, coordinates_.altitude);
            boost::hash_combine(seed, name_);
            return seed;
        }

        inline const CoordGeodetic &coordinates() const noexcept {
            return coordinates_;
        }

        inline const std::string &name() const noexcept {
            return name_;
        }

    private:
        CoordGeodetic coordinates_;

        std::string name_;
    };

    void to_json(nlohmann::json &json, const GroundStation &station);

    void from_json(const nlohmann::json &json, GroundStation &station);
}

namespace std {
    inline ostream &operator<<(ostream &out, const quake::GroundStation &station) {
        out << station.name();
        return out;
    }

    template<>
    struct hash<quake::GroundStation> {
        std::size_t operator()(const quake::GroundStation &station) const noexcept {
            return station.hash();
        }
    };
}

#endif //QUAKE_GROUND_STATION_H
