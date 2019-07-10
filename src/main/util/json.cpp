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

#include "json.h"

void boost::posix_time::to_json(nlohmann::json &json, const boost::posix_time::ptime &value) {
    json = boost::posix_time::to_simple_string(value);
}

void boost::posix_time::from_json(const nlohmann::json &json, boost::posix_time::ptime &value) {
    value = boost::posix_time::time_from_string(json.get<std::string>());
}

void boost::posix_time::to_json(nlohmann::json &json, const boost::posix_time::time_duration &value) {
    json = boost::posix_time::to_simple_string(value);
}

void boost::posix_time::from_json(const nlohmann::json &json, boost::posix_time::time_duration &value) {
    value = boost::posix_time::duration_from_string(json.get<std::string>());
}

void boost::posix_time::to_json(nlohmann::json &json, const boost::posix_time::time_period &value) {
    nlohmann::json object;
    object["begin"] = value.begin();
    object["end"] = value.end();
    json = object;
}

void boost::posix_time::from_json(const nlohmann::json &json, boost::posix_time::time_period &value) {
    value = {json.at("begin").get<boost::posix_time::ptime>(), json.at("end").get<boost::posix_time::ptime>()};
}

template<>
boost::posix_time::time_period quake::util::from_json<boost::posix_time::time_period>(const nlohmann::json &json) {
    auto time_period = util::DefaultPeriod();
    boost::posix_time::from_json(json, time_period);
    return time_period;
}
