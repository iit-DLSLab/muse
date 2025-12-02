/* CPYHDR { */
/*
 * This file is part of the 'iit-rbd' library.
 * Copyright © 2015 2016 2017, Marco Frigerio (marco.frigerio@iit.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef IIT_ROBCOGEN_TEST_DYNAMICS_H_
#define IIT_ROBCOGEN_TEST_DYNAMICS_H_

#include "../utils.h"

namespace iit {
namespace robcogen {
namespace test {

/**
 * \name Command-line tests of the dynamics.
 *
 * Very simple "tests", which parse the argc/argv arguments available in a
 * main(), and print to standard output the result of a dynamics algorithm.
 *
 * The command line arguments are expected to be enough to initialize all the
 * joint-status vectors required as input by the dynamics algorithm.
 */
///@{
template<class RT>
void cmdline_fixedBase_invdyn(
        int argc,
        char** argv,
        typename RT::InvDynEngine& id)
{
    typename RT::JointState q, qd, qdd, tau;
    utils::cmdlineargs_jstate<RT>(argc-1, &(argv[1]), q, qd, qdd);
    id.id(tau, q,qd,qdd);
    std::cout << tau << std::endl;
}

template<class RT>
void cmdline_fixedBase_fwddyn(
        int argc,
        char** argv,
        typename RT::FwdDynEngine& fd)
{
    typename RT::JointState q, qd, qdd, tau;
    utils::cmdlineargs_jstate<RT>(argc-1, &(argv[1]), q, qd, tau);
    fd.fd(qdd, q,qd,tau);
    std::cout << qdd << std::endl;
}

template<class RT>
void cmdline_jsim(
        int argc,
        char** argv,
        typename RT::JSIM& jsim)
{
    typename RT::JointState q;
    utils::cmdlineargs_jstate<RT>(argc-1, &(argv[1]), q);
    std::cout << jsim(q) << std::endl;
}
///@}

}
}
}

#endif
