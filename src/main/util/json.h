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

#ifndef QUAKE_JSON_H
#define QUAKE_JSON_H

#include <nlohmann/json.hpp>

#include <glog/logging.h>

#include <boost/config.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#include "util/datetime.h"

namespace boost {

    namespace posix_time {

        void to_json(nlohmann::json &json, const ptime &value);

        void from_json(const nlohmann::json &json, ptime &value);

        void to_json(nlohmann::json &json, const time_duration &value);

        void from_json(const nlohmann::json &json, time_duration &value);

        void to_json(nlohmann::json &json, const time_period &value);

        void from_json(const nlohmann::json &json, time_period &value);
    }
}


namespace quake {

    namespace util {

        template<typename ValueType>
        ValueType from_json(const nlohmann::json &json) {
            return json.get<ValueType>();
        }

        template<>
        boost::posix_time::time_period from_json(const nlohmann::json &json);

        template<typename ObjectType>
        void Save(const ObjectType &object, const boost::filesystem::path &output_path) {
            std::ofstream output_stream;

            output_stream.open(output_path.string(), std::ofstream::out);
            LOG_IF(FATAL, !output_stream.is_open()) << "Failed to save json object in " << output_path << " file.";

            nlohmann::json json_object = object;
            output_stream << json_object;

            output_stream.close();
            LOG_IF(FATAL, output_stream.is_open());
        }
    }
}


#endif //QUAKE_JSON_H
