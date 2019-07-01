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

#include "ground_station.h"

#include <boost/algorithm/string.hpp>

#include <memory>
#include <unordered_map>

quake::GroundStation::GroundStation(CoordGeodetic coordinates, std::string name)
        : coordinates_(coordinates),
          name_(std::move(name)) {}

// elevation obtained using the Coordinates applet at http://dateandtime.info/citycoordinates.php and https://www.latlong.net/
const quake::GroundStation quake::GroundStation::London{{51.509865, -0.118092, 0.015}, "London"};
const quake::GroundStation quake::GroundStation::Glasgow{{55.8642, -4.2518, 0.040}, "Glasgow"};
const quake::GroundStation quake::GroundStation::Thurso{{58.5936, -3.5221, 0.022}, "Thurso"};
const quake::GroundStation quake::GroundStation::Manchester{{53.4808, -2.2426, 0.051}, "Manchester"};
const quake::GroundStation quake::GroundStation::Birmingham{{52.4862, -1.8904, 0.149}, "Birmingham"};
const quake::GroundStation quake::GroundStation::Bristol{{51.4545, -2.5879, 0.011}, "Bristol"};
const quake::GroundStation quake::GroundStation::Ipswich{{52.0567, 1.1482, 0.018}, "Ipswich"};
const quake::GroundStation quake::GroundStation::Cambridge{{52.2053, 0.1218, 0.012}, "Cambridge"};
const quake::GroundStation quake::GroundStation::York{{53.9600, -1.0873, 0.017}, "York"};
const quake::GroundStation quake::GroundStation::None{{0.0, 0.0, 0.0}, "None"};

const std::vector<quake::GroundStation> quake::GroundStation::All = {
        quake::GroundStation::London,
        quake::GroundStation::Glasgow,
        quake::GroundStation::Thurso,
        quake::GroundStation::Manchester,
        quake::GroundStation::Birmingham,
        quake::GroundStation::Bristol,
        quake::GroundStation::Ipswich,
        quake::GroundStation::Cambridge,
        quake::GroundStation::York
};

const quake::GroundStation &quake::GroundStation::FromName(const std::string &name) {
    const auto &ground_station = FromNameOrNone(name);
    if (ground_station == quake::GroundStation::None) {
        std::stringstream error;
        error << "Station " << name << " not found.";
        throw std::runtime_error(error.str());
    }
    return ground_station;
}

const quake::GroundStation &quake::GroundStation::FromNameOrNone(const std::string &name) {
    static const std::unordered_map<std::string, quake::GroundStation> OPTIONS = {
            {"london",     GroundStation::London},
            {"glasgow",    GroundStation::Glasgow},
            {"thurso",     GroundStation::Thurso},
            {"manchester", GroundStation::Manchester},
            {"birmingham", GroundStation::Birmingham},
            {"bristol",    GroundStation::Bristol},
            {"ipswich",    GroundStation::Ipswich},
            {"cambridge",  GroundStation::Cambridge},
            {"york",       GroundStation::York}
    };

    const auto name_to_use = boost::to_lower_copy(name);
    auto found_it = OPTIONS.find(name_to_use);
    if (found_it == std::cend(OPTIONS)) {
        return GroundStation::None;
    }

    return found_it->second;
}

std::ostream &operator<<(std::ostream &out, const quake::GroundStation &station) {
    out << station.name();
    return out;
}

void quake::to_json(nlohmann::json &json, const quake::GroundStation &station) {
    json = station.name();
}
