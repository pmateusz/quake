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

#ifndef QUAKE_VALIDATION_H
#define QUAKE_VALIDATION_H

#include <string>

namespace quake {

    namespace util {

        bool validate_input_file(const char *flagname, const std::string &input);

        bool validate_output_file(const char *flagname, const std::string &output);

        bool validate_duration(const char *flagname, const std::string &duration);

        bool validate_date(const char *flagname, const std::string &date);

        template<typename ValueType>
        class IntervalConstraintedValue {
        public:
            IntervalConstraintedValue(ValueType lower_bound, ValueType upper_bound)
                    : lower_bound_{lower_bound},
                      upper_bound_{upper_bound} {}

            bool operator()(ValueType value) const {
                return value <= upper_bound_ and lower_bound_ <= value;
            }

        private:
            ValueType lower_bound_;
            ValueType upper_bound_;
        };
    }
}


#endif //QUAKE_VALIDATION_H
