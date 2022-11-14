//
// Created by samuel rac on 11/3/19.
//

#ifndef BUZZ_QPSOLVER_H
#define BUZZ_QPSOLVER_H

#include "alglib/stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "alglib/optimization.h"
#include <sys/time.h>

using namespace alglib;


class QPSolver {
private:
    std::vector<double> m_A;
    std::vector<double> m_b;
    std::vector<double> m_C;
    std::vector<double> m_d;
    std::vector<double> m_bndl;
    std::vector<double> m_bndu;
    std::vector<double> m_solution;

public:

    QPSolver(const std::vector<double> &mA, const std::vector<double> &mB, const std::vector<double> &mC);

    QPSolver(const std::vector<double> &mA, const std::vector<double> &mB, const std::vector<double> &mC,
             const std::vector<double> &mBndl, const std::vector<double> &mBndu);

    virtual ~QPSolver();

    std::vector<double> solve();

    const std::vector<double> &get_solution() const;
};


#endif //BUZZ_QPSOLVER_H
