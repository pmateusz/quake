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

#include <iostream>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "gurobi_c++.h"

#include "util/logging.h"

#include "problem_generator.h"

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    quake::util::SetupLogging(argv[0]);

    std::vector<quake::GroundStation> ground_stations{quake::GroundStation::Glasgow,
                                                      quake::GroundStation::London,
                                                      quake::GroundStation::Cambridge,
                                                      quake::GroundStation::Thurso,
                                                      quake::GroundStation::Bristol,
                                                      quake::GroundStation::Ipswich,
                                                      quake::GroundStation::York,
                                                      quake::GroundStation::Birmingham,
                                                      quake::GroundStation::Manchester};
    std::unordered_map<quake::GroundStation, double> coefficients = quake::ProblemGenerator::GetLinearDistributionCoefficients(
            ground_stations);

    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    std::vector<std::vector<GRBVar> > weights(ground_stations.size(),
                                              std::vector<GRBVar>(ground_stations.size(), GRBVar{}));

    auto ZERO = model.addVar(0, 0, 0, GRB_CONTINUOUS);
    for (decltype(ground_stations.size()) row = 0; row < ground_stations.size(); ++row) {
        for (decltype(row) column = 0; column < ground_stations.size(); ++column) {
            weights[row][column] = model.addVar(0, 1, 0, GRB_CONTINUOUS);
            weights[column][row] = model.addVar(0, 1, 0, GRB_CONTINUOUS);
        }
    }

#define GLASGOW 0
#define LONDON 1
#define CAMBRIDGE 2
#define THURSO 3
#define BRISTOL 4
#define IPSWICH 5
#define YORK 6
#define BIRMINGHAM 7
#define MANCHESTER 8

    model.addConstr(weights[LONDON][BRISTOL] >= 0.05);
    model.addConstr(weights[LONDON][CAMBRIDGE] >= 0.04);
    model.addConstr(weights[LONDON][IPSWICH] >= 0.05);
    model.addConstr(weights[CAMBRIDGE][BIRMINGHAM] >= 0.01);
    model.addConstr(weights[MANCHESTER][BIRMINGHAM] >= 0.05);
    model.addConstr(weights[MANCHESTER][YORK] >= 0.03);
    model.addConstr(weights[MANCHESTER][BRISTOL] >= 0.03);
    model.addConstr(weights[GLASGOW][LONDON] >= 0.04);
    model.addConstr(weights[GLASGOW][BIRMINGHAM] >= 0.04);
    model.addConstr(weights[GLASGOW][YORK] >= 0.02);
    model.addConstr(weights[GLASGOW][THURSO] >= 0.01);
    model.addConstr(weights[GLASGOW][MANCHESTER] >= 0.01);

    // diagonal is zero
    for (decltype(ground_stations.size()) row = 0; row < ground_stations.size(); ++row) {
        model.addConstr(weights[row][row] == 0);
    }

    // symmetry
    for (decltype(ground_stations.size()) row = 0; row < ground_stations.size(); ++row) {
        for (decltype(row) column = 0; column < ground_stations.size(); ++column) {
            model.addConstr(weights[row][column] == weights[column][row]);
        }
    }

    // constraint incoming flow
    for (decltype(ground_stations.size()) row = 0; row < ground_stations.size(); ++row) {
        GRBLinExpr incoming_flow = 0;
        for (decltype(row) column = 0; column < ground_stations.size(); ++column) {
            incoming_flow += weights[row][column];
        }
        model.addConstr(incoming_flow <= coefficients.at(ground_stations[row]));
    }

    // set objective
    GRBLinExpr obj;
    for (decltype(ground_stations.size()) row = 0; row < ground_stations.size(); ++row) {
        for (decltype(row) column = 0; column < row; ++column) {
            obj += weights[row][column];
        }
    }
    model.setObjective(obj, GRB_MAXIMIZE);

    model.optimize();

    for (decltype(ground_stations.size()) row = 0; row < ground_stations.size(); ++row) {
        for (decltype(row) column = 0; column < row; ++column) {
            std::cout << std::setw(10) << ground_stations[row].name()
                      << " <-> "
                      << std::setw(10) << ground_stations[column].name()
                      << ": " << weights[row][column].get(GRB_DoubleAttr_X) << std::endl;
        }
    }

    return 0;
}
