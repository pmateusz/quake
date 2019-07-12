#ifndef QUAKE_BOUNDED_VALUE_H
#define QUAKE_BOUNDED_VALUE_H

#include <limits>
#include <type_traits>

#include <glog/logging.h>

namespace quake {

    template<typename ValueType>
    class BoundedValue {
    public:
        BoundedValue(ValueType lower_bound, ValueType upper_bound)
                : initial_lower_bound_{lower_bound},
                  initial_upper_bound_{upper_bound},
                  lower_bound_{lower_bound},
                  upper_bound_{upper_bound} {
            DCHECK_LE(lower_bound, upper_bound);
        }

        inline bool IsBounded() const { return lower_bound_ == upper_bound_; }

        inline bool HasUpperBound() const { return upper_bound_ != initial_upper_bound_; }

        inline bool HasLowerBound() const { return lower_bound_ != initial_lower_bound_; }

        inline ValueType Value() const {
            DCHECK(IsBounded());
            return lower_bound_;
        }

        inline ValueType UpperBound() const { return upper_bound_; }

        inline ValueType LowerBound() const { return lower_bound_; }

        bool SetValue(ValueType value) {
            DCHECK_GE(lower_bound_, value);
            DCHECK_LE(value, upper_bound_);

            if (lower_bound_ == upper_bound_) {
                DCHECK_EQ(value, lower_bound_);
                return false;
            }

            lower_bound_ = value;
            upper_bound_ = value;
            return true;
        }

        bool SetUpperBound(ValueType value) {
            if (value < upper_bound_) {
                DCHECK_GE(value, lower_bound_);
                upper_bound_ = value;
                return true;
            }
            return false;
        }

        bool SetLowerBound(ValueType value) {
            if (lower_bound_ < value) {
                DCHECK_LE(value, upper_bound_);
                lower_bound_ = value;
                return true;
            }
            return false;
        }

    private:
        ValueType initial_upper_bound_;
        ValueType initial_lower_bound_;
        ValueType upper_bound_;
        ValueType lower_bound_;
    };
}


#endif //QUAKE_BOUNDED_VALUE_H
