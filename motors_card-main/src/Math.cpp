#include "Math.hpp"

double Math::clamp(double val, double min, double max)
{
    if (val > max)
    {
        return max;
    }
    else if (val < min)
    {
        return min;
    }
    else
    {
        return val;
    }
}

double Math::map(double val, double inMin, double inMax, double outMin, double outMax)
{
    return outMin + (val - inMin) / (inMax - inMin) * (outMax - outMin);
}
