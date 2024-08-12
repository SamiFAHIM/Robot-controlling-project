#include "PID.hpp"

#include "Math.hpp"


PIDparams default_params;

PID::PID(double minOutput, double maxOutput) : m_pid_params(default_params),
                                               m_minOutput(minOutput),
                                               m_maxOutput(maxOutput)
{
}

double PID::compute(uint32_t dt)
{
    double proportional;
    double integral;
    double derivative;

    m_errors.error = m_setpoint - m_processValue;

    if (!m_errors.lastErrorSet)
    {
        m_errors.lastError = m_errors.error;
        m_errors.lastErrorSet = true;
    }

    proportional = m_pid_params.Kp * m_errors.error;

    // TODO(cjaverliat): Use trapezoidal integration instead
    m_errors.accumulator += m_errors.error * dt / 1000;
    integral = Math::clamp(m_pid_params.Ki * m_errors.accumulator, m_minOutput, m_maxOutput);

    if (dt == 0)
    {
        derivative = 0;
    }
    else
    {
        derivative = m_pid_params.Kd * (m_errors.error - m_errors.lastError) / dt * 1000;
    }

    m_errors.lastError = m_errors.error;

    return Math::clamp(proportional + integral + derivative, m_minOutput, m_maxOutput);
}

void PID::setSetpoint(double setpoint) { m_setpoint = setpoint; }

double PID::getSetpoint() { return m_setpoint; }

void PID::setGains(PIDparams pid_params)
{
    m_pid_params = pid_params;
}

void PID::setProcessValue(double processValue) { m_processValue = processValue; }

void PID::reset()
{
    m_errors = {0, 0, false, 0};
}
