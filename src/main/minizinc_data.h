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

#include <string>
#include <vector>
#include <unordered_map>

#ifndef QUAKE_MINIZINC_DATA_H
#define QUAKE_MINIZINC_DATA_H

namespace quake {

    class MiniZincRange {
    public:
        MiniZincRange(std::string name, int from, int to);

        const std::string &name() const;

        int from() const;

        int to() const;

        int size() const;

    private:
        std::string name_;
        int from_;
        int to_;
    };

    template<typename IntegralType>
    class MiniZincData {
    public:
        static const std::string STATION_REF;
        static const std::string DUMMY_STATION_REF;
        static const std::string TIME_REF;
        static const std::string SWITCH_DURATION_REF;
        static const std::string STEP_DURATION_REF;
        static const std::string TRANSFER_SHARE_REF;
        static const std::string INITIAL_BUFFER_REF;
        static const std::string KEY_CONSUMPTION_REF;
        static const std::string KEY_RATE_REF;
        static const std::string KEY_RATE_CUMUL_REF;
        static const std::string TIME_OFFSET_REF;
        static const std::string START_TIME_REF;
        static const std::string END_TIME_REF;

        MiniZincData(std::unordered_map<std::string, std::vector<std::string> > enum_sets,
                     std::unordered_map<std::string, MiniZincRange> ranges,
                     std::unordered_map<std::string, std::string> enums,
                     std::unordered_map<std::string, std::string> strings,
                     std::unordered_map<std::string, IntegralType> ints,
                     std::unordered_map<std::string, double> floats,
                     std::unordered_map<std::string, std::vector<IntegralType> > arrays1d_of_int,
                     std::unordered_map<std::string, std::vector<std::vector<IntegralType> > > arrays2d_of_int,
                     std::unordered_map<std::string, std::vector<double> > arrays1d_of_float,
                     std::unordered_map<std::string, std::vector<std::vector<double> > > arrays2d_of_float);

        MiniZincData(const MiniZincData &other);

        MiniZincData(MiniZincData &&other) noexcept;

        MiniZincData &operator=(MiniZincData &&other) noexcept;

        MiniZincData &operator=(const MiniZincData &other) noexcept;

        const MiniZincRange Range(const std::string &name) const;

        const std::string &String(const std::string &name) const;

        const std::string &Enum(const std::string &name) const;

        const std::vector<std::string> &EnumSet(const std::string &name) const;

        double Float(const std::string &name) const;

        IntegralType Int(const std::string &name) const;

        const std::vector<IntegralType> &Array1dOfInt(const std::string &name) const;

        const std::vector<double> &Array1dOfDouble(const std::string &name) const;

        const std::vector<std::vector<IntegralType> > &Array2dOfInt(const std::string &name) const;

        const std::vector<std::vector<double> > &Array2dOfDouble(const std::string &name) const;

    private:
        std::unordered_map<std::string, std::vector<std::string> > enum_sets_;
        std::unordered_map<std::string, std::string> enums_;
        std::unordered_map<std::string, MiniZincRange> ranges_;
        std::unordered_map<std::string, std::string> strings_;
        std::unordered_map<std::string, double> floats_;
        std::unordered_map<std::string, IntegralType> ints_;

        std::unordered_map<std::string, std::vector<IntegralType> > arrays1d_of_int_;
        std::unordered_map<std::string, std::vector<std::vector<IntegralType> > > arrays2d_of_int_;

        std::unordered_map<std::string, std::vector<double> > arrays1d_of_float_;
        std::unordered_map<std::string, std::vector<std::vector<double> > > arrays2d_of_float_;
    };
}

namespace quake {

    template<typename IntegralType>
    const std::vector<std::string> &quake::MiniZincData<IntegralType>::EnumSet(std::string const &name) const {
        return enum_sets_.at(name);
    }

    template<typename IntegralType>
    const quake::MiniZincRange quake::MiniZincData<IntegralType>::Range(const std::string &name) const {
        return ranges_.at(name);
    }

    template<typename IntegralType>
    const std::string &quake::MiniZincData<IntegralType>::String(const std::string &name) const {
        return strings_.at(name);
    }

    template<typename IntegralType>
    const std::string &quake::MiniZincData<IntegralType>::Enum(const std::string &name) const {
        return enums_.at(name);
    }

    template<typename IntegralType>
    double quake::MiniZincData<IntegralType>::Float(const std::string &name) const {
        return floats_.at(name);
    }

    template<typename IntegralType>
    IntegralType quake::MiniZincData<IntegralType>::Int(const std::string &name) const {
        return ints_.at(name);
    }

    template<typename IntegralType>
    const std::vector<IntegralType> &
    quake::MiniZincData<IntegralType>::Array1dOfInt(const std::string &name) const {
        return arrays1d_of_int_.at(name);
    }

    template<typename IntegralType>
    const std::vector<double> &quake::MiniZincData<IntegralType>::Array1dOfDouble(const std::string &name) const {
        return arrays1d_of_float_.at(name);
    }

    template<typename IntegralType>
    const std::vector<std::vector<IntegralType> > &
    quake::MiniZincData<IntegralType>::Array2dOfInt(const std::string &name) const {
        return arrays2d_of_int_.at(name);
    }

    template<typename IntegralType>
    const std::vector<std::vector<double> > &
    quake::MiniZincData<IntegralType>::Array2dOfDouble(const std::string &name) const {
        return arrays2d_of_float_.at(name);
    }

    template<typename IntegralType>
    quake::MiniZincData<IntegralType>::MiniZincData(
            std::unordered_map<std::string, std::vector<std::string> > enum_sets,
            std::unordered_map<std::string, quake::MiniZincRange> ranges,
            std::unordered_map<std::string, std::string> enums,
            std::unordered_map<std::string, std::string> strings,
            std::unordered_map<std::string, IntegralType> ints,
            std::unordered_map<std::string, double> floats,
            std::unordered_map<std::string, std::vector<IntegralType> > arrays1d_of_int,
            std::unordered_map<std::string, std::vector<std::vector<IntegralType> > > arrays2d_of_int,
            std::unordered_map<std::string, std::vector<double> > arrays1d_of_float,
            std::unordered_map<std::string, std::vector<std::vector<double> > > arrays2d_of_float)
            : enum_sets_(std::move(enum_sets)),
              ranges_(std::move(ranges)),
              enums_(std::move(enums)),
              strings_(std::move(strings)),
              ints_(std::move(ints)),
              floats_(std::move(floats)),
              arrays1d_of_int_(std::move(arrays1d_of_int)),
              arrays1d_of_float_(std::move(arrays1d_of_float)),
              arrays2d_of_int_(std::move(arrays2d_of_int)),
              arrays2d_of_float_(std::move(arrays2d_of_float)) {}

    template<typename IntegralType>
    quake::MiniZincData<IntegralType>::MiniZincData(quake::MiniZincData<IntegralType> &&other) noexcept
            : enum_sets_(std::move(other.enum_sets_)),
              ranges_(std::move(other.ranges_)),
              enums_(std::move(other.enums_)),
              strings_(std::move(other.strings_)),
              ints_(std::move(other.ints_)),
              floats_(std::move(other.floats_)),
              arrays1d_of_int_(std::move(other.arrays1d_of_int_)),
              arrays1d_of_float_(std::move(other.arrays1d_of_float_)),
              arrays2d_of_int_(std::move(other.arrays2d_of_int_)),
              arrays2d_of_float_(std::move(other.arrays2d_of_float_)) {}

    template<typename IntegralType>
    quake::MiniZincData<IntegralType> &
    quake::MiniZincData<IntegralType>::operator=(quake::MiniZincData<IntegralType> &&other) noexcept {
        enum_sets_ = std::move(other.enum_sets_);
        ranges_ = std::move(other.ranges_);
        enums_ = std::move(other.enums_);
        strings_ = std::move(other.strings_);
        ints_ = std::move(other.ints_);
        floats_ = std::move(other.floats_);
        arrays1d_of_int_ = std::move(other.arrays1d_of_int_);
        arrays1d_of_float_ = std::move(other.arrays1d_of_float_);
        arrays2d_of_int_ = std::move(other.arrays2d_of_int_);
        arrays2d_of_float_ = std::move(other.arrays2d_of_float_);
        return *this;
    }

    template<typename IntegralType>
    quake::MiniZincData<IntegralType> &
    quake::MiniZincData<IntegralType>::operator=(const quake::MiniZincData<IntegralType> &other) noexcept {
        enum_sets_ = other.enum_sets_;
        ranges_ = other.ranges_;
        enums_ = other.enums_;
        strings_ = other.strings_;
        ints_ = other.ints_;
        floats_ = other.floats_;
        arrays1d_of_int_ = other.arrays1d_of_int_;
        arrays1d_of_float_ = other.arrays1d_of_float_;
        arrays2d_of_int_ = other.arrays2d_of_int_;
        arrays2d_of_float_ = other.arrays2d_of_float_;
        return *this;
    }

    template<typename IntegralType>
    quake::MiniZincData<IntegralType>::MiniZincData(const quake::MiniZincData<IntegralType> &other)
            : enum_sets_(other.enum_sets_),
              ranges_(other.ranges_),
              enums_(other.enums_),
              strings_(other.strings_),
              ints_(other.ints_),
              floats_(other.floats_),
              arrays1d_of_int_(other.arrays1d_of_int_),
              arrays1d_of_float_(other.arrays1d_of_float_),
              arrays2d_of_int_(other.arrays2d_of_int_),
              arrays2d_of_float_(other.arrays2d_of_float_) {}
}

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::STATION_REF = "STATION";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::DUMMY_STATION_REF = "dummy_station";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::TIME_REF = "TIME";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::SWITCH_DURATION_REF = "switch_duration";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::STEP_DURATION_REF = "step_duration";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::TRANSFER_SHARE_REF = "transfer_share";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::INITIAL_BUFFER_REF = "initial_buffer";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::KEY_CONSUMPTION_REF = "key_consumption";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::KEY_RATE_REF = "key_rate";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::KEY_RATE_CUMUL_REF = "key_rate_cumul";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::TIME_OFFSET_REF = "time_offset";

template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::START_TIME_REF = "start_time";


template<typename IntegralType>
const std::string quake::MiniZincData<IntegralType>::END_TIME_REF = "end_time";
#endif //QUAKE_MINIZINC_DATA_H
