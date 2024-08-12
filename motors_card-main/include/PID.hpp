#ifndef PID_HPP
#define PID_HPP

#include "PID.hpp"
#include "params.h"
#include "Arduino.h"

/**
 * Proportional-integral-derivative regulator class.
 *
 * Given a setpoint and a process value, will return the regulated value when
 * calling compute. The integral factor and the output is clamped between
 * minOutput and maxOutput.
 *
 * Errors can be reset by calling resetErrors.
 */
class PID
{
public:
    struct Errors
    {
        double error = 0;

        double lastError = 0;
        bool lastErrorSet = false;

        double accumulator = 0;
    };

    PID(double minOutput, double maxOutput);

    /**
     * Compute the regulated value depending on the errors.
     *
     * t = t + dt
     * e(t) = setpoint - process_value(t)
     * p(t) = Kp * e(t)
     * i(t) = Ki * \int_{0}^{t} e(t)dt
     * d(t) = Kd * (e(t) - e(t - dt)) / dt
     *
     * output(t) = p(t) + i(t) + d(t)
     */
    double compute(uint32_t dt);

    /**
     * Goal the PID is trying to reach.
     *
     * Should be positive. If you need to set a negative setpoint, use the
     * absolute value and handle the sign elsewhere.
     */
    void setSetpoint(double setpoint);

    double getSetpoint();

    void setGains(PIDparams pid_params);
    /**
     * Current (physical) value reached by the system.
     *
     * Should be positive. If you need to set a negative setpoint, use the
     * absolute value and handle the sign elsewhere.
     */
    void setProcessValue(double processValue);

    /**
     * Reset all the errors of the PID.
     *
     * This is mostly used to reset integral accumulator.
     */
    void reset();

private:
    PIDparams m_pid_params;

    PID::Errors m_errors;

    double m_setpoint;
    double m_processValue;

    double m_minOutput;
    double m_maxOutput;
};

#endif
