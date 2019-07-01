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

#include "base_interval.h"


// TODO: develop hashcode

quake::BaseInterval::BaseInterval()
        : BaseInterval(0, 0, 0) {}

quake::BaseInterval::BaseInterval(std::size_t station_index, std::size_t begin, std::size_t end)
        : StationIndex{station_index},
          Begin{begin},
          End{end} {}

bool quake::BaseInterval::operator==(const quake::BaseInterval &other) const {
    return Begin == other.Begin
           && End == other.End
           && StationIndex == other.StationIndex;
}

std::ostream &operator<<(std::ostream &out, const quake::BaseInterval &interval) {
    out << typeid(interval).name() << " station="
        << interval.StationIndex << " interval=["
        << interval.Begin << ", "
        << interval.End << "] duration="
        << interval.End - interval.Begin;
    return out;
}

quake::VarInterval::VarInterval() : VarInterval({}, 0, 0, 0) {}

quake::VarInterval::VarInterval(GRBVar var, std::size_t station_index, std::size_t begin, std::size_t end)
        : BaseInterval(station_index, begin, end),
          Var{var} {}

quake::VarInterval quake::VarInterval::Create(GRBModel &model, const quake::BaseInterval &prototype) {
    return {model.addVar(0.0, 1.0, 0.0, GRB_BINARY, (boost::format("station%1%_%2%_%3%")
                                                     % prototype.StationIndex
                                                     % prototype.Begin
                                                     % prototype.End).str()),
            prototype.StationIndex,
            prototype.Begin,
            prototype.End};
}