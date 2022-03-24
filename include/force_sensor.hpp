#ifndef DELTA_FORCE_SENSOR_HPP
#define DELTA_FORCE_SENSOR_HPP

template <class THardware>
class ForceSensor {
    double sampling_frequency_;

public:
    explicit ForceSensor(double sampling_frequency);
};

#endif //DELTA_FORCE_SENSOR_HPP
