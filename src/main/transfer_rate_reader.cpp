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

#include "transfer_rate_reader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

#include <glog/logging.h>

inline std::runtime_error OnSemicolonExpected() {
    return std::runtime_error{"Invalid format. Semicolon expected."};
}

inline std::runtime_error OnInvalidFormat() {
    return std::runtime_error{"Invalid format."};
}

std::vector<std::tuple<double, double, double> >
quake::TransferRateReader::Read(const boost::filesystem::path &file_path) const {
    std::ifstream stream(file_path.string());
    if (!stream.is_open()) {
        std::stringstream msg;
        msg << "Failed to open file " << file_path << ".";
        throw std::invalid_argument(msg.str());
    }

    std::string line;
    if (!getline(stream, line)) {
        throw std::runtime_error("Failed to read headers");
    }

    const auto line_lowercase = boost::to_lower_copy(line);
    if (line_lowercase != "elevation;rate of secrete keys;rate of secrete bits") {
        throw std::runtime_error("Invalid format. Unrecognized columns in header.");
    }

    std::vector<std::tuple<double, double, double> > results;
    while (getline(stream, line)) {
        std::istringstream input_line{line};

        double elevation, key_rate, bit_rate;
        input_line >> elevation;

        if (!input_line) {
            throw OnInvalidFormat();
        }
        if (input_line.get() != ';') {
            throw OnSemicolonExpected();
        }

        input_line >> key_rate;
        if (!input_line) {
            throw OnInvalidFormat();
        }
        if (input_line.get() != ';') {
            throw OnSemicolonExpected();
        }

        input_line >> bit_rate;
        if (!input_line) {
            throw OnInvalidFormat();
        }

        results.emplace_back(std::make_tuple(elevation, key_rate, bit_rate));
    }

    return results;
}
