#ifndef MATH_HPP
#define MATH_HPP

namespace Math
{

    /**
     * Keep the val in the range [min; max]
     */
    double clamp(double val, double min, double max);

    /**
     * Map a value for the input range [inMin; inMax] to the output range [outMin;
     * outMax]
     */
    double map(double val, double inMin, double inMax, double outMin, double outMax);

} // namespace Math

#endif
