//
// Created by samuel rac on 11/3/19.
//

#include "rosbuzz/bezier_utils/QPSolver.h"


QPSolver::QPSolver(const std::vector<double> &mA, const std::vector<double> &mB, const std::vector<double> &mC) : m_A(
        mA), m_b(mB), m_C(mC) {}

QPSolver::QPSolver(const std::vector<double> &mA, const std::vector<double> &mB, const std::vector<double> &mC,
                   const std::vector<double> &mBndl, const std::vector<double> &mBndu) : m_A(mA), m_b(mB), m_C(mC),
                                                                                         m_bndl(mBndl), m_bndu(mBndu) {}

QPSolver::~QPSolver() {
    m_A.clear();
    m_b.clear();
    m_C.clear();
    m_d.clear();
    m_bndu.clear();
    m_bndl.clear();
    m_solution.clear();
}

std::vector<double> QPSolver::solve()
{
    // To measure the time of the function
    // static struct timeval t1, t2;
    // gettimeofday(&t1, NULL);
    
    real_2d_array a;
    a.setcontent(sqrt(m_A.size()), sqrt(m_A.size()), m_A.data());
    real_1d_array b;
    b.setcontent(m_b.size(), m_b.data());
//    real_1d_array s = "[1,1]";
//    int s_size = sqrt(m_A.size());
//    long s_data[s_size];
//    for (int i=0; i < s_size; i++)
//        s_data[i]=1;
//    integer_1d_array s;
//    s.setcontent(s_size, s_data);
    real_2d_array c;
    c.setcontent(m_C.size()/(sqrt(m_A.size())+1), sqrt(m_A.size())+1, m_C.data());
    int ct_size = m_C.size() / (sqrt(m_A.size()) + 1);
    long ct_data[ct_size];
    for (int i=0; i < ct_size; i++)
        ct_data[i]=-1;
    integer_1d_array ct;
    ct.setcontent(ct_size, ct_data);
    real_1d_array x;
    minqpstate state;
    minqpreport rep;

    // create solver, set quadratic/linear terms
    int problem_size = sqrt(m_A.size());
    minqpcreate(problem_size, state);
    minqpsetquadraticterm(state, a);
    minqpsetlinearterm(state, b);
    minqpsetlc(state, c, ct);
    // if bounds are set
    if (!m_bndu.empty() && !m_bndl.empty()) {
        real_1d_array bndl;
        bndl.setcontent(m_bndl.size(), m_bndl.data());
        real_1d_array bndu;
        bndu.setcontent(m_bndu.size(), m_bndu.data());
        minqpsetbc(state, bndl, bndu);
    }

    // Set scale of the parameters.
    // minqpsetscale(state, s);
    minqpsetscaleautodiag(state);


    // Solve problem with BLEIC-based QP solver.
//    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
//    minqpoptimize(state);
//    minqpresults(state, x, rep);
//    printf("%s\n", x.tostring(1).c_str()); // EXPECTED: [1.500,0.500]

    //
    // Solve problem with DENSE-AUL solver.
    // This is the one used for the experiment
    minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 10);
    
    // Solve problem with DENSE-IPM solver.
    // Also this give the flag of convergence!!
    // minqpsetalgodenseipm(state, 1.0e-9);
    
    // Solve problem with BLEIC-based QP solver.
    // This is the one that actually report the correct flag of convergence!
    // minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);

    // Solve problem with QUICK-QP
    // minqpsetalgoquickqp(state, 0.0, 0.0, 0.0, 0, false);

    
    minqpoptimize(state);
    minqpresults(state, x, rep);
    //printf("Result: %s\n", x.tostring(1).c_str());
    //printf("Code exit: %d\n", (int) rep.terminationtype);

    // double lambda_2;
    // lambda_2 = m_C[2] / 100.0 + 0.1;

    // if (lambda_2 < 0.1)
    //    printf("================AAAAAAAAAAAAAA=================\n");
    // // Calculate unfeasibility
    // if ((m_C[0]*x[0] + m_C[1]*x[1]) > m_C[2])
    // {
        
    //     printf("----ERROR---- lambda_2 = %f\n", lambda_2);
    // }

    // if ((m_C[3]*x[0] + m_C[4]*x[1]) > m_C[5])
    // {
        
    //     printf("---------------BBBBBB-------------------\n");
    // }

    // if ((m_C[6]*x[0] + m_C[7]*x[1]) > m_C[8])
    // {
        
    //     printf("---------------CCCCCCC-------------------\n");
    // }



    for(int i=0; i<x.length(); i++)
        m_solution.push_back(x(i));

    /* std::vector<double> rep_vector;
    for(int i=0; i<rep.length(); i++)
        rep_vector.push_back(x(i)); */
    if (((int) rep.terminationtype != 1) && ((int) rep.terminationtype != 2) && ((int) rep.terminationtype != 3) && ((int) rep.terminationtype != 4))
        printf("Code exit: %d\n", (int) rep.terminationtype);    

    // gettimeofday(&t2, NULL);
    // double elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec); //[us]
    // printf("QP elapsed time: %f [us]\n", elapsedTime);
    
    return m_solution;
}

const std::vector<double> &QPSolver::get_solution() const {
    return m_solution;
}
