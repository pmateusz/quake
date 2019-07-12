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

#ifndef QUAKE_GENERATE_MINI_ZINC_DATA_COMMAND_H
#define QUAKE_GENERATE_MINI_ZINC_DATA_COMMAND_H

#include "command.h"

#include <vector>

#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#include "ground_station.h"

namespace quake {

    class GenerateMiniZincDataCommand : public Command {
    public:
        GenerateMiniZincDataCommand(boost::posix_time::time_period time_period,
                                    boost::posix_time::time_duration time_step,
                                    boost::filesystem::path output,
                                    std::vector<GroundStation> ground_stations,
                                    bool convert_float_to_int);

        void Run() override;

    private:
        const boost::posix_time::time_period time_period_;
        const boost::posix_time::time_duration time_step_;
        const boost::filesystem::path output_;
        std::vector<GroundStation> ground_stations_;
        const bool convert_float_to_int_;
    };
}


#endif //QUAKE_GENERATE_MINI_ZINC_DATA_COMMAND_H
