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

#ifndef QUAKE_BASE_INTERVAL_H
#define QUAKE_BASE_INTERVAL_H

#include <gurobi_c++.h>

#include <boost/config.hpp>
#include <boost/format.hpp>
#include <boost/functional/hash.hpp>

namespace quake {

    struct BaseInterval {
        BaseInterval();

        BaseInterval(std::size_t station_index, std::size_t begin, std::size_t end);

        bool operator==(const quake::BaseInterval &other) const;

        std::size_t StationIndex;
        std::size_t Begin;
        std::size_t End;
    };

    struct VarInterval : public BaseInterval {
        VarInterval();

        VarInterval(GRBVar var, std::size_t station_index, std::size_t begin, std::size_t end);

        static VarInterval Create(GRBModel &model, const BaseInterval &prototype);

        GRBVar Var;
    };
}

std::ostream &operator<<(std::ostream &out, const quake::BaseInterval &interval);

namespace std {

    template<>
    struct hash<quake::BaseInterval> {
        std::size_t operator()(const quake::BaseInterval &interval) const {
            std::size_t seed = 0;

            boost::hash_combine(seed, boost::hash_value(interval.StationIndex));
            boost::hash_combine(seed, boost::hash_value(interval.Begin));
            boost::hash_combine(seed, boost::hash_value(interval.End));

            return seed;
        }
    };
}

#endif //QUAKE_BASE_INTERVAL_H
