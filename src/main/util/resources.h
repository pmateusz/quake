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


#ifndef QUAKE_RESOURCES_H
#define QUAKE_RESOURCES_H

#include <boost/filesystem.hpp>

#include <wordexp.h>

namespace quake {

    class GroundStation;

    namespace util {

        class Resources {
        public:
            static const std::string DEFAULT_LOCAL_TIME_ZONE;

            explicit Resources(boost::filesystem::path data_directory);

            boost::filesystem::path SunsetSunriseData(const GroundStation &station) const;

            boost::filesystem::path GetElevationData(const std::string &file_name) const;

            boost::filesystem::path TransferRate(int rate) const;

            boost::filesystem::path GetMiniZincData(const std::string &file_name) const;

            boost::filesystem::path GetCloudCoverData(const std::string &file_name) const;

        private:
            struct wordexp_holder {
                wordexp_holder() = default;

                ~wordexp_holder();

                wordexp_t value;
            };

            boost::filesystem::path data_directory_;
        };
    }
}


#endif //QUAKE_RESOURCES_H
