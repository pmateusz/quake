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

#ifndef QUAKE_MINIZINC_READER_H
#define QUAKE_MINIZINC_READER_H

#include <glog/logging.h>
#include <boost/regex.hpp>

#include <regex>
#include <string>
#include <exception>
#include <functional>

#include <boost/algorithm/string.hpp>

#include "minizinc_data.h"

namespace quake {

    template<typename InputStreamType, typename IntegralType>
    class MiniZincReader {
    public:
        explicit MiniZincReader(InputStreamType &&stream) noexcept;

        MiniZincReader(const MiniZincReader &other) = delete;

        MiniZincReader &operator=(const MiniZincReader &other) = delete;

        MiniZincReader(MiniZincReader &&other) noexcept;

        MiniZincReader &operator=(MiniZincReader &&other) noexcept;

        MiniZincData <IntegralType> Read();

        void Close();

        virtual ~MiniZincReader();

    private:
        InputStreamType input_stream_;

        void TryClose();

        static std::vector<double> ToArrayOfDouble(const std::string &line);

        static std::vector<IntegralType> ToArrayOfInt(const std::string &line);

        static std::vector<std::string> SplitRemoveEmpty(const std::string &line);

        template<typename ValueType, typename CallbackType>
        std::vector<std::vector<ValueType> >
        ToArray2d(const std::string &current_line, CallbackType callback);
    };
}

static std::runtime_error OnFailedToReadFile() {
    return std::runtime_error("Failed to read a file");
}

static const auto ARRAY1D_PATTERN = boost::regex("^\\[(.*?)\\]$");
static const auto ARRAY2D_OPEN_PATTERN = boost::regex("^([\\w_]+)\\s+=\\s+\\[\\s*\\|\\s*(.*?)$");
static const auto ARRAY2D_CONTINUE_PATTERN = boost::regex("^\\s*\\|\\s*(.*?)$");
static const auto ARRAY2D_CLOSE_PATTERN = boost::regex("^\\s*\\|\\s*(.*?)\\s*\\|\\s*\\]\\s*;$");

namespace quake {
    template<typename InputStreamType, typename IntegralType>
    std::vector<double> MiniZincReader<InputStreamType, IntegralType>::ToArrayOfDouble(const std::string &line) {
        std::vector<double> result;
        std::vector<std::string> raw_results = SplitRemoveEmpty(line);
        for (const auto &raw_result : raw_results) {
            result.emplace_back(std::stod(raw_result));
        }
        return result;
    }

    template<typename InputStreamType, typename IntegralType>
    MiniZincData <IntegralType> MiniZincReader<InputStreamType, IntegralType>::Read() {
        static const auto ASSIGNMENT_PATTERN = boost::regex("^([\\w_]+)\\s+=\\s+(.*?);$");
        static const auto MULTI_LINE_ASSIGNMENT_PATTERN = boost::regex("\\s*[\\w_]+\\s*=(.*?)$");
        static const auto INT_PATTERN = boost::regex("^[-+]?\\d+$");
        static const auto FLOAT_PATTERN = boost::regex("^[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?$");
        static const auto FLOAT_CHUNK_PATTERN = boost::regex("\\d+\\.");
        static const auto STRING_PATTERN = boost::regex("^\"(.*?)\"$");
        static const auto ENUM_DECLARATION_PATTERN = boost::regex("^\\{(.*?)\\}$");
        static const auto ENUM_PATTERN = boost::regex("^([\\w_]+)$");
        static const auto ENUM_NAME_PATTERN = boost::regex("([\\w_]+)");
        static const auto RANGE_PATTERN = boost::regex("^([+-]?\\d+)\\.\\.([+-]?\\d+)$");

        std::unordered_map<std::string, std::string> string_vars;
        std::unordered_map<std::string, std::string> enum_vars;
        std::unordered_map<std::string, quake::MiniZincRange> range_vars;
        std::unordered_map<std::string, IntegralType> int_vars;
        std::unordered_map<std::string, double> float_vars;
        std::unordered_map<std::string, std::vector<std::string> > enum_declarations;
        std::unordered_map<std::string, std::vector<IntegralType> > array1d_of_int_vars;
        std::unordered_map<std::string, std::vector<double> > array1d_of_double_vars;
        std::unordered_map<std::string, std::vector<std::vector<IntegralType> > > array2d_of_int_vars;
        std::unordered_map<std::string, std::vector<std::vector<double> > > array2d_of_double_vars;

        std::string line;
        boost::smatch match_results;
        while (std::getline(input_stream_, line)) {
            if (boost::regex_match(line, match_results, ASSIGNMENT_PATTERN)) {
                std::string name{match_results[1]};
                std::string payload{match_results[2]};

                if (boost::regex_match(payload, match_results, STRING_PATTERN)) {
                    std::string payload_to_use{match_results[1]};
                    string_vars.emplace(std::move(name), std::move(payload_to_use));
                } else if (boost::regex_match(payload, match_results, INT_PATTERN)) {
                    int_vars.emplace(std::move(name), std::stoi(payload));
                } else if (boost::regex_match(payload, match_results, FLOAT_PATTERN)) {
                    float_vars.emplace(std::move(name), std::stod(payload));
                } else if (boost::regex_match(payload, match_results, ENUM_DECLARATION_PATTERN)) {
                    std::string payload_without_braces = match_results[1];
                    auto enum_set = SplitRemoveEmpty(payload_without_braces);
                    enum_declarations.emplace(std::move(name), std::move(enum_set));
                } else if (boost::regex_match(payload, match_results, ENUM_PATTERN)) {
                    enum_vars.emplace(std::move(name), match_results[1]);
                } else if (boost::regex_match(payload, match_results, RANGE_PATTERN)) {
                    int left = std::stoi(match_results[1]);
                    int right = std::stoi(match_results[2]);
                    range_vars.emplace(name, MiniZincRange(name, left, right));
                } else if (boost::regex_match(payload, match_results, ARRAY1D_PATTERN)) {
                    std::string payload_without_braces = match_results[1];
                    if (boost::regex_search(payload_without_braces, match_results, FLOAT_CHUNK_PATTERN)) {
                        array1d_of_double_vars.emplace(std::move(name), std::move(
                                MiniZincReader<InputStreamType, IntegralType>::ToArrayOfDouble(payload_without_braces)));
                    } else {
                        array1d_of_int_vars.emplace(name,
                                                    MiniZincReader<InputStreamType, IntegralType>::ToArrayOfInt(
                                                            payload_without_braces));
                    }
                }
            } else if (boost::regex_match(line, match_results, ARRAY2D_OPEN_PATTERN)) {
                std::string name{match_results[1]};
                std::string payload{match_results[2]};

                if (boost::regex_search(payload, match_results, FLOAT_CHUNK_PATTERN)) {
                    array2d_of_double_vars.emplace(std::move(name),
                                                   std::move(ToArray2d<double>(
                                                           payload,
                                                           MiniZincReader<InputStreamType, IntegralType>::ToArrayOfDouble)));
                } else {
                    array2d_of_int_vars.emplace(std::move(name),
                                                std::move(ToArray2d<IntegralType>(
                                                        payload,
                                                        MiniZincReader<InputStreamType, IntegralType>::ToArrayOfInt)));
                }
            }
        }

        if (input_stream_.bad()) {
            throw OnFailedToReadFile();
        }

        return {
                std::move(enum_declarations),
                std::move(range_vars),
                std::move(enum_vars),
                std::move(string_vars),
                std::move(int_vars),
                std::move(float_vars),
                std::move(array1d_of_int_vars),
                std::move(array2d_of_int_vars),
                std::move(array1d_of_double_vars),
                std::move(array2d_of_double_vars)
        };
    }

    template<typename InputStreamType, typename IntegralType>
    MiniZincReader<InputStreamType, IntegralType>::MiniZincReader(InputStreamType &&stream) noexcept
            : input_stream_(std::move(stream)) {}

    template<typename InputStreamType, typename IntegralType>
    MiniZincReader<InputStreamType, IntegralType>::MiniZincReader(MiniZincReader &&other) noexcept
            : input_stream_(std::move(other.input_stream_)) {}

    template<typename InputStreamType, typename IntegralType>
    MiniZincReader<InputStreamType, IntegralType> &
    MiniZincReader<InputStreamType, IntegralType>::operator=(MiniZincReader &&other) noexcept {
        input_stream_ = std::move(other.input_stream_);
        return *this;
    }

    template<typename InputStreamType, typename IntegralType>
    void MiniZincReader<InputStreamType, IntegralType>::TryClose() {
        if (input_stream_.is_open()) {
            input_stream_.close();
        }
    }

    template<typename InputStreamType, typename IntegralType>
    MiniZincReader<InputStreamType, IntegralType>::~MiniZincReader() {
        TryClose();
    }

    template<typename InputStreamType, typename IntegralType>
    void MiniZincReader<InputStreamType, IntegralType>::Close() {
        input_stream_.close();
    }

    template<typename InputStreamType, typename IntegralType>
    std::vector<IntegralType> MiniZincReader<InputStreamType, IntegralType>::ToArrayOfInt(const std::string &line) {
        std::vector<IntegralType> result;
        std::vector<std::string> raw_results = SplitRemoveEmpty(line);
        for (const auto &raw_result : raw_results) {
            result.emplace_back(std::stoi(raw_result));
        }
        return result;
    }

    template<typename InputStreamType, typename IntegralType>
    std::vector<std::string> MiniZincReader<InputStreamType, IntegralType>::SplitRemoveEmpty(const std::string &line) {
        std::vector<std::string> raw_tokens;
        boost::split(raw_tokens, line, [](char c) -> bool { return std::isspace(c) || c == ','; });
        std::vector<std::string> result;
        for (auto &token: raw_tokens) {
            if (token.empty()) { continue; }
            result.emplace_back(std::move(token));
        }
        return result;
    }

    template<typename InputStreamType, typename IntegralType>
    template<typename ValueType, typename CallbackType>
    std::vector<std::vector<ValueType> >
    MiniZincReader<InputStreamType, IntegralType>::ToArray2d(const std::string &first_line, CallbackType callback) {
        std::vector<std::vector<ValueType> > results;

        results.emplace_back(std::move(callback(first_line)));

        boost::smatch matcher;
        std::string current_line;
        while (std::getline(input_stream_, current_line)) {
            if (boost::regex_match(current_line, matcher, ARRAY2D_CLOSE_PATTERN)) {
                std::string data = matcher[1];
                results.emplace_back(std::move(callback(data)));
                break;
            } else if (boost::regex_match(current_line, matcher, ARRAY2D_CONTINUE_PATTERN)) {
                std::string data = matcher[1];
                results.emplace_back(std::move(callback(data)));
            } else {
                DCHECK(false) << "Code should be unreachable";
            }
        }

        if (input_stream_.bad()) {
            throw OnFailedToReadFile();
        }

        if (!results.empty()) {
            const auto size = results.size();
            const auto expected_dim = results[0].size();
            for (auto pos = 1; pos < size; ++pos) {
                CHECK_EQ(expected_dim, results[pos].size());
            }
        }
        return results;
    }
}

#endif //QUAKE_MINIZINC_READER_H
