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

#ifndef QUAKE_MINIZINC_DATA_MODEL_H
#define QUAKE_MINIZINC_DATA_MODEL_H

#include <vector>
#include <string>
#include <locale>
#include <type_traits>

#include <boost/date_time.hpp>

#include <glog/logging.h>

#include "minizinc_data.h"

namespace quake {

    template<typename IntegralType>
    class CoreMiniZincDataModel {
    public:
        explicit CoreMiniZincDataModel(quake::MiniZincData<IntegralType> data_model)
                : data_model_(std::move(data_model)) {
            static const boost::posix_time::time_input_facet *INPUT_FACET = new boost::posix_time::time_input_facet{
                    "%Y-%b-%d %H:%M:%S"};
            static const std::locale LOCALE(std::locale::classic(), INPUT_FACET);

            std::string raw_start_time = data_model_.String(quake::MiniZincData<IntegralType>::START_TIME_REF);
            CHECK(ParseDateTime(raw_start_time, &start_time_)) << "Failed to parse end time: " << raw_start_time << ".";

            std::string raw_end_time = data_model_.String(quake::MiniZincData<IntegralType>::END_TIME_REF);
            CHECK(ParseDateTime(raw_end_time, &end_time_)) << "Failed to parse end time: " << raw_end_time << ".";
        }

        const boost::posix_time::ptime &StartTime() const { return start_time_; }

        const boost::posix_time::ptime &EndTime() const { return end_time_; }

        const std::vector<std::string> &Stations() const {
            return data_model_.EnumSet(quake::MiniZincData<IntegralType>::STATION_REF);
        }

        const std::string &DummyStation() const {
            return data_model_.Enum(quake::MiniZincData<IntegralType>::DUMMY_STATION_REF);
        }

        IntegralType SwitchDuration() const {
            return data_model_.Int(quake::MiniZincData<IntegralType>::SWITCH_DURATION_REF);
        }

        IntegralType StepDuration() const {
            return data_model_.Int(MiniZincData<IntegralType>::STEP_DURATION_REF);
        }

        const std::vector<double> &TransferShare() const {
            return data_model_.Array1dOfDouble(MiniZincData<IntegralType>::TRANSFER_SHARE_REF);
        }

        const std::vector<IntegralType> &InitialBuffer() const {
            return data_model_.Array1dOfInt(MiniZincData<IntegralType>::INITIAL_BUFFER_REF);
        }

        const std::vector<IntegralType> &KeyConsumption() const {
            return data_model_.Array1dOfInt(MiniZincData<IntegralType>::KEY_CONSUMPTION_REF);
        }

        const std::vector<IntegralType> &TimeOffset() const {
            return data_model_.Array1dOfInt(MiniZincData<IntegralType>::TIME_OFFSET_REF);
        }

        quake::MiniZincRange Time() const {
            return data_model_.Range(MiniZincData<IntegralType>::TIME_REF);
        }

        boost::posix_time::ptime TimeAt(int step) const {
            return start_time_ + boost::posix_time::seconds(step);
        }

    protected:
        bool ParseDateTime(const std::string &value, boost::posix_time::ptime *date_time) const;

        boost::posix_time::ptime start_time_;
        boost::posix_time::ptime end_time_;

        MiniZincData<IntegralType> data_model_;
    };

    template<typename IntegralType>
    bool CoreMiniZincDataModel<IntegralType>::ParseDateTime(const std::string &value,
                                                            boost::posix_time::ptime *date_time) const {
        static const boost::posix_time::time_input_facet *INPUT_FACET
                = new boost::posix_time::time_input_facet{"%Y-%b-%d %H:%M:%S"};
        static const std::locale LOCALE(std::locale::classic(), INPUT_FACET);

        std::stringstream stream{value};
        stream.imbue(LOCALE);
        stream >> *date_time;
        return stream.good();
    }

    template<typename KeyRateType, typename IntegralType, typename Enable = void>
    class MiniZincDataModel : public CoreMiniZincDataModel<IntegralType> {
    public:
        explicit MiniZincDataModel(MiniZincData<IntegralType> data_model)
                : CoreMiniZincDataModel<IntegralType>(std::move(data_model)) {}

        const std::vector<std::vector<KeyRateType> > &KeyRate() const;

        const std::vector<std::vector<KeyRateType> > &KeyRateCumulative() const;
    };

    template<typename KeyRateType, typename IntegralType>
    class MiniZincDataModel<KeyRateType, IntegralType, typename std::enable_if<std::is_floating_point<KeyRateType>::value>::type>
            : public CoreMiniZincDataModel<IntegralType> {
    public:
        explicit MiniZincDataModel(MiniZincData<IntegralType> data_model)
                : CoreMiniZincDataModel<IntegralType>(std::move(data_model)) {}

        const std::vector<std::vector<KeyRateType> > &KeyRate() const {
            return this->data_model_.Array2dOfDouble(MiniZincData<IntegralType>::KEY_RATE_REF);
        }

        const std::vector<std::vector<KeyRateType> > &KeyRateCumulative() const {
            return this->data_model_.Array2dOfDouble(MiniZincData<IntegralType>::KEY_RATE_CUMUL_REF);
        }
    };

    template<typename KeyRateType, typename IntegralType>
    class MiniZincDataModel<KeyRateType, IntegralType, typename std::enable_if<std::is_integral<KeyRateType>::value>::type>
            : public CoreMiniZincDataModel<IntegralType> {
    public:
        explicit MiniZincDataModel(MiniZincData<IntegralType> data_model)
                : CoreMiniZincDataModel<IntegralType>(std::move(data_model)) {}

        const std::vector<std::vector<KeyRateType> > &KeyRate() const {
            return this->data_model_.Array2dOfInt(MiniZincData<IntegralType>::KEY_RATE_REF);
        }

        const std::vector<std::vector<KeyRateType> > &KeyRateCumulative() const {
            return this->data_model_.Array2dOfInt(MiniZincData<IntegralType>::KEY_RATE_CUMUL_REF);
        }
    };
}

#endif //QUAKE_MINIZINC_DATA_MODEL_H
