#ifndef QUAKE_DATETIME_H
#define QUAKE_DATETIME_H

#include <boost/config.hpp>
#include <boost/date_time.hpp>

namespace quake {

    namespace util {

        inline boost::posix_time::time_period DefaultPeriod() {
            return {boost::posix_time::ptime(), boost::posix_time::seconds(0)};
        }
    }
}
#endif //QUAKE_DATETIME_H
