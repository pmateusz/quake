//
// Copyright 2019 Mateusz Polnik
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

#include "math.h"

#include <glog/logging.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>

boost::numeric::ublas::vector<double> quake::util::mean(const boost::numeric::ublas::matrix<double> &matrix) {
    boost::numeric::ublas::vector<double> mean(matrix.size2());

    for (auto col = 0; col < matrix.size2(); ++col) {
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean> > acc;
        for (auto row = 0; row < matrix.size1(); ++row) {
            acc(matrix(row, col));
        }
        const auto point_wise_mean = boost::accumulators::mean(acc);
        mean(col) = point_wise_mean;
    }

    return mean;
}

boost::numeric::ublas::matrix<double> quake::util::covariance(const boost::numeric::ublas::matrix<double> &matrix) {
    CHECK_GE(matrix.size1(), 1);

    boost::numeric::ublas::matrix<double> cov_matrix(matrix.size2(), matrix.size2());
    const auto mean_vector = mean(matrix);
    for (auto col1 = 0; col1 < matrix.size2(); ++col1) {
        for (auto col2 = 0; col2 < matrix.size2(); ++col2) {
            double acc = 0.0;
            for (auto row = 0; row < matrix.size1(); ++row) {
                acc += (matrix(row, col1) - mean_vector(col1)) * (matrix(row, col2) - mean_vector(col2));
            }

            double unbiased_covariance;
            if (matrix.size1() == 1) {
                unbiased_covariance = acc;
            } else {
                unbiased_covariance = acc / (matrix.size1() - 1);
            }
            cov_matrix(col1, col2) = unbiased_covariance;
            cov_matrix(col2, col1) = unbiased_covariance;
        }
    }

    return cov_matrix;
}
