#ifndef DELTA_DEFINES_HPP
#define DELTA_DEFINES_HPP

struct ConvergenceParameters {
    double x_abstol;
    double x_reltol;
    double f_x_abstol;
    double f_x_reltol;
    unsigned int max_iterations;
};

#endif //DELTA_DEFINES_HPP
