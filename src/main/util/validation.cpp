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

#include "validation.h"

#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

bool quake::util::validate_input_file(const char *flagname, const std::string &input) {
    if (input.empty()) {
        return true;
    }

    if (!boost::filesystem::exists(input)) {
        LOG(ERROR) << "The input file " << input << " does not exists.";
        return false;
    }
    return true;
}

bool quake::util::validate_output_file(const char *flagname, const std::string &output) {
    if (output.empty()) {
        return true;
    }

    if (boost::filesystem::exists(output)) {
        LOG(ERROR) << "The output file " << output << " already exists.";
        return false;
    }
    return true;
}

bool quake::util::validate_duration(const char *flagname, const std::string &duration) {
    if (duration.empty()) {
        return true;
    }

    boost::posix_time::duration_from_string(duration);
    return true;
}

bool quake::util::validate_date(const char *flagname, const std::string &date) {
    if (date.empty()) {
        return true;
    }

    boost::gregorian::from_simple_string(date);
    return true;
}
