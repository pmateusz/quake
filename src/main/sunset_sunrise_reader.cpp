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

#include "sunset_sunrise_reader.h"
#include "ground_station.h"

#include <unordered_map>
#include <fstream>
#include <regex>
#include <string>
#include <chrono>
#include <sstream>
#include <stdexcept>

#include <glog/logging.h>

#include <boost/algorithm/string.hpp>

#include <date/tz.h>

const std::regex quake::SunsetSunriseReader::GROUND_STATION_PATTERN{"\\.?\\s+(\\d+)\\s+.*?\\s+for\\s+(\\w+)\\s+"
                                                                    "longitude\\s+(\\w)(\\d+)\\s+(\\d+)\\s+"
                                                                    "latitude\\s+(\\w)(\\d+)\\s+(\\d+)",
                                                                    std::regex_constants::icase};
const std::regex quake::SunsetSunriseReader::EMPTY_LINE_PATTERN{"^\\s+$"};
const std::regex quake::SunsetSunriseReader::TIMES_FORMAT_PATTERN{"^\\s+times\\s+in\\s+gmt$",
                                                                  std::regex_constants::icase};
const std::regex quake::SunsetSunriseReader::FIRST_6_HEADER_MONTHS_PATTERN{"\\s+january\\s+february\\s+march"
                                                                           "\\s+april\\s+may\\s+june",
                                                                           std::regex_constants::icase};
const std::regex quake::SunsetSunriseReader::SECOND_6_MONTHS_HEADER_PATTERN{"\\s+july\\s+august\\s+september"
                                                                            "\\s+october\\s+november\\s+december",
                                                                            std::regex_constants::icase};
const std::regex quake::SunsetSunriseReader::HOUR_MINUTE_HEADER_PATTERN{"(\\s+h\\s+m){12}",
                                                                        std::regex_constants::icase};
const std::regex quake::SunsetSunriseReader::RISE_SET_HEADER_PATTERN{"(\\s+rise\\s+set){6}",
                                                                     std::regex_constants::icase};

const std::unordered_map<std::string, quake::GroundStation> quake::SunsetSunriseReader::GROUND_STATIONS_BY_NAME{
        {"cambridge",  GroundStation::Cambridge},
        {"birmingham", GroundStation::Birmingham},
        {"bristol",    GroundStation::Bristol},
        {"glasgow",    GroundStation::Glasgow},
        {"ipswich",    GroundStation::Ipswich},
        {"london",     GroundStation::London},
        {"manchester", GroundStation::Manchester},
        {"thurso",     GroundStation::Thurso},
        {"york",       GroundStation::York},
        {"belfast",    GroundStation::Belfast}
};

quake::SunsetSunriseReader::SunsetSunriseReader(std::string local_time_zone_name)
        : local_time_zone_name_{std::move(local_time_zone_name)},
          local_time_zone_ptr_{date::locate_zone(local_time_zone_name_)},
          utc_time_zone_ptr_{date::locate_zone("UTC")} {}

bool RiseSetComparator(std::pair<boost::posix_time::ptime, boost::posix_time::ptime> left,
                       std::pair<boost::posix_time::ptime, boost::posix_time::ptime> right) {
    return left.first.date() < right.first.date();
}

std::pair<quake::GroundStation, std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > >
quake::SunsetSunriseReader::Read(const boost::filesystem::path &file_path) const {
    std::ifstream stream(file_path.string());
    if (!stream.is_open()) {
        std::stringstream msg;
        msg << "Failed to open file " << file_path << ".";
        throw std::invalid_argument(msg.str());
    }

    std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > results;
    auto year = 0;
    auto ground_station = GroundStation::None;
    for (std::string line; getline(stream, line);) {
        std::smatch match_result;
        if (std::regex_match(line, match_result, GROUND_STATION_PATTERN)) {
            year = std::stoi(match_result[1].str(), nullptr, 10);

            std::string ground_station_name = match_result[2].str();
            boost::algorithm::to_lower(ground_station_name);
            auto find_it = GROUND_STATIONS_BY_NAME.find(ground_station_name);
            if (find_it == std::end(GROUND_STATIONS_BY_NAME)) {
                std::stringstream msg;
                msg << "Failed to find ground station " << match_result[2].str() << ".";
                throw std::domain_error(msg.str());
            }
            if (ground_station == GroundStation::None) {
                ground_station = find_it->second;
            } else if (ground_station != find_it->second) {
                std::stringstream msg;
                msg << "Data from 2 ground stations found in a single file: "
                    << ground_station.name() << " and "
                    << find_it->second.name() << ".";
                throw std::domain_error(msg.str());
            }

            auto partial_results = ReadSunsetSunrise(stream, year);
            sort(std::begin(partial_results), std::end(partial_results), RiseSetComparator);
            std::copy(std::cbegin(partial_results), std::cend(partial_results), std::back_inserter(results));
        }
    }

    return std::make_pair(ground_station, std::move(results));
}

std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> >
quake::SunsetSunriseReader::ReadSunsetSunrise(std::ifstream &stream, int year) const {
    std::smatch match_result;
    for (std::string line; getline(stream, line);) {
        if (std::regex_match(line, match_result, EMPTY_LINE_PATTERN)
            || std::regex_match(line, match_result, TIMES_FORMAT_PATTERN)
            || line.empty()) {
            continue;
        }

        if (std::regex_match(line, match_result, FIRST_6_HEADER_MONTHS_PATTERN)) {
            return ReadSunsetSunrise(stream, 1, 6, year);
        }

        if (std::regex_match(line, match_result, SECOND_6_MONTHS_HEADER_PATTERN)) {
            return ReadSunsetSunrise(stream, 7, 12, year);
        }

        throw std::domain_error("Invalid format. Expected names of months in the header.");
    }

    throw std::domain_error("Invalid format. Expected names of months in the header. No more input.");
}

int SkipSpaces(std::istringstream &stream) {
    auto counter = 0;
    while (stream.good() && std::isspace(stream.peek())) {
        stream.ignore(1);
        ++counter;
    }
    return counter;
}

std::domain_error OnDataAlignmentError() {
    return std::domain_error("Invalid format. Unexpected data alignment");
}

template<typename Clock, typename Duration>
date::time_of_day<Duration> to_time_of_day(std::chrono::time_point<Clock, Duration> tp) {
    auto local_date = date::floor<date::days>(tp);
    return date::make_time(tp - local_date);
}

std::pair<boost::posix_time::ptime, boost::posix_time::ptime>
quake::SunsetSunriseReader::ReadDay(int day, int month, int year, std::istringstream &stream) const {
    int rise_hour, rise_minute, set_hour, set_minute;
    stream >> rise_hour;
    auto skipped_chars = SkipSpaces(stream);
    if (skipped_chars != 1) {
        throw OnDataAlignmentError();
    }
    stream >> rise_minute;

    skipped_chars = SkipSpaces(stream);
    if (skipped_chars != 2) {
        throw OnDataAlignmentError();
    }
    stream >> set_hour;

    skipped_chars = SkipSpaces(stream);
    if (skipped_chars != 1) {
        throw OnDataAlignmentError();
    }
    stream >> set_minute;

    const date::year_month_day today = date::year(year) / month / day;
    CHECK(today.ok());

    const auto local_rise_time = date::make_zoned(local_time_zone_ptr_,
                                                  date::local_days{today}
                                                  + std::chrono::hours(rise_hour)
                                                  + std::chrono::minutes(rise_minute));

    const auto local_set_time = date::make_zoned(local_time_zone_ptr_,
                                                 date::local_days{today}
                                                 + std::chrono::hours(set_hour)
                                                 + std::chrono::minutes(set_minute));

    const auto utc_rise_time = date::make_zoned(utc_time_zone_ptr_, local_rise_time);
    const auto utc_set_time = date::make_zoned(utc_time_zone_ptr_, local_set_time);

    const auto utc_rise_tod = to_time_of_day(utc_rise_time.get_sys_time());
    const auto utc_set_tod = to_time_of_day(utc_set_time.get_sys_time());

    const auto date = boost::gregorian::date(year, month, day);
    return std::make_pair(
            boost::posix_time::ptime(date,
                                     boost::posix_time::time_duration(utc_rise_tod.hours().count(),
                                                                      utc_rise_tod.minutes().count(),
                                                                      0)),
            boost::posix_time::ptime(date,
                                     boost::posix_time::time_duration(utc_set_tod.hours().count(),
                                                                      utc_set_tod.minutes().count(),
                                                                      0)));
}

std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> >
quake::SunsetSunriseReader::ReadSunsetSunrise(std::ifstream &stream, int first_month, int last_month, int year) const {
    std::string line;
    std::smatch match_result;

    auto passed_rise_set_header = false;
    while (getline(stream, line)) {
        if (std::regex_match(line, match_result, EMPTY_LINE_PATTERN) || line.empty()) {
            continue;
        }

        if (std::regex_match(line, match_result, RISE_SET_HEADER_PATTERN)) {
            passed_rise_set_header = true;
            break;
        }

        throw std::domain_error("Invalid format. Expected rise and set in the header.");
    }

    if (!passed_rise_set_header) {
        throw std::domain_error("Invalid format. Expected rise and set in the header. No more input.");
    }

    auto passed_hour_minute_header = false;
    while (getline(stream, line)) {
        if (std::regex_match(line, match_result, EMPTY_LINE_PATTERN) || line.empty()) {
            continue;
        }

        if (std::regex_match(line, match_result, HOUR_MINUTE_HEADER_PATTERN)) {
            passed_hour_minute_header = true;
            break;
        }

        throw std::domain_error("Invalid format. Expected hour and minute in the header.");
    }

    if (!passed_hour_minute_header) {
        throw std::domain_error("Invalid format. Expected hour and minute in the header. No more input.");
    }

    std::vector<std::pair<boost::posix_time::ptime, boost::posix_time::ptime> > result;

    static const auto MAX_DAYS_IN_MONTH = 31;
    static const auto MONTH_SEPARATOR_COUNT = 3;
    static const auto MONTH_CHAR_COUNT = 15;

    auto last_day_passed = 0;
    while (getline(stream, line) && (last_day_passed < MAX_DAYS_IN_MONTH)) {
        if (std::regex_match(line, match_result, EMPTY_LINE_PATTERN) || line.empty()) {
            continue;
        }

        const auto original_size = line.size();
        auto characters_processed = 0;
        std::istringstream input_line(line);

        int current_day;
        input_line >> current_day;

        ++last_day_passed;
        if (current_day != last_day_passed) {
            throw std::domain_error("Invalid format. Expected a different day of the month.");
        }

        auto current_month = first_month;
        while (input_line.good() && current_month <= last_month) {
            auto skipped_chars = SkipSpaces(input_line);
            if (skipped_chars == MONTH_SEPARATOR_COUNT) {
                auto day_data = ReadDay(current_day, current_month, year, input_line);
                result.push_back(day_data);
                ++current_month;
            } else {
                if (skipped_chars < MONTH_CHAR_COUNT) {
                    throw OnDataAlignmentError();
                }

                auto months_to_skip = skipped_chars / MONTH_CHAR_COUNT;
                current_month += months_to_skip;
                if (current_month > last_month) {
                    break;
                }

                skipped_chars -= months_to_skip * MONTH_CHAR_COUNT;
                if (skipped_chars == MONTH_SEPARATOR_COUNT) {
                    auto day_data = ReadDay(current_day, current_month, year, input_line);
                    result.push_back(day_data);
                    ++current_month;
                } else {
                    throw OnDataAlignmentError();
                }
            }
        }

        if (!input_line.eof()) {
            throw std::domain_error("Invalid format. A line is too long.");
        }

        if (current_month <= last_month && (current_month != 6)) {
            throw std::domain_error("Invalid format. Failed to load data for all months.");
        }
    }

    if (last_day_passed != MAX_DAYS_IN_MONTH) {
        throw std::domain_error("Invalid format. Failed to load data for all days in a month.");
    }
    return result;
}
