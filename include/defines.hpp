#ifndef DELTA_DEFINES_HPP
#define DELTA_DEFINES_HPP

struct ConvergenceParameters {
    double x_abstol;
    double x_reltol;
    double f_x_abstol;
    double f_x_reltol;
    unsigned int max_iterations;

    explicit ConvergenceParameters(
            double x_abstol = std::numeric_limits<double>::epsilon(),
            double x_reltol = std::numeric_limits<double>::epsilon(),
            double f_x_abstol = std::numeric_limits<double>::epsilon(),
            double f_x_reltol = std::numeric_limits<double>::epsilon(),
            unsigned int max_iterations = 100)
            : x_abstol(x_abstol), x_reltol(x_reltol), f_x_abstol(f_x_abstol), f_x_reltol(f_x_reltol),
            max_iterations(max_iterations) {

    }
};



#endif //DELTA_DEFINES_HPP
