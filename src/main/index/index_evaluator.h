#ifndef QUAKE_INDEX_EVALUATOR_H
#define QUAKE_INDEX_EVALUATOR_H

#include <vector>

#include "extended_problem.h"

namespace quake {

    class Forecast;

    class Solution;

    class IndexEvaluator {
    public:
        explicit IndexEvaluator(const ExtendedProblem &problem);

        double operator()(const Solution &solution) const;

        double operator()(const Solution &solution, const Forecast &forecast) const;

        double WorstCase(const Solution &solution, const std::vector<Forecast> &forecasts) const;

    private:
        double Evaluate(const Solution &solution,
                        const std::function<double(const GroundStation &, const boost::posix_time::time_period &)> &key_rate) const;

        const ExtendedProblem &problem_;
    };
}


#endif //QUAKE_INDEX_EVALUATOR_H
