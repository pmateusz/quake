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


#include <wordexp.h>

#include <stdexcept>
#include <sstream>
#include <memory>

#include "resources.h"
#include "ground_station.h"

quake::util::Resources::wordexp_holder::~wordexp_holder() {
    wordfree(&value);
}

const std::string quake::util::Resources::DEFAULT_LOCAL_TIME_ZONE{"Europe/London"};

quake::util::Resources::Resources(boost::filesystem::path data_directory) {
    wordexp_holder exp_holder{};

    const auto status_code = wordexp(data_directory.c_str(), &exp_holder.value, 0);
    if (status_code != 0) {
        std::stringstream error_msg;
        error_msg << "Failed to resolve the filepath " << data_directory
                  << ". Returned the error code " << status_code << ".";
        throw std::invalid_argument(error_msg.str());
    }

    data_directory_ = boost::filesystem::canonical(*exp_holder.value.we_wordv);
}

boost::filesystem::path quake::util::Resources::SunsetSunriseData(const quake::GroundStation &station) const {
    std::stringstream file_name;
    file_name << station.name() << ".txt";
    return data_directory_ / "sunset_sunrise" / file_name.str();
}

boost::filesystem::path quake::util::Resources::GetElevationData(const std::string &file_name) const {
    return data_directory_ / "elevation" / file_name;
}

boost::filesystem::path quake::util::Resources::TransferRate(int rate) const {
    return data_directory_ / "key_rate" / ("data_24_05eff_" + std::to_string(rate) + ".csv");
}

boost::filesystem::path quake::util::Resources::GetMiniZincData(const std::string &file_name) const {
    return data_directory_ / "minizinc" / file_name;
}

boost::filesystem::path quake::util::Resources::GetCloudCoverData(const std::string &file_name) const {
    return data_directory_ / "cloud_cover" / file_name;
}
