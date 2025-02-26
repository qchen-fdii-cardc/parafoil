#ifndef MATH_UTIL_HPP
#define MATH_UTIL_HPP

#include <vector>
#include <valarray>
#include <iostream>

namespace parafoil
{

    // Scalar multiplication for vector
    inline std::vector<double> operator*(const std::vector<double> &v, double scalar)
    {
        std::vector<double> result(v.size());
        for (size_t i = 0; i < v.size(); ++i)
        {
            result[i] = v[i] * scalar;
        }
        return result;
    }

    // Scalar multiplication (commutative)
    inline std::vector<double> operator*(double scalar, const std::vector<double> &v)
    {
        return v * scalar;
    }

    // Vector addition
    inline std::vector<double> operator+(const std::vector<double> &v1, const std::vector<double> &v2)
    {
        std::vector<double> result(v1.size());
        for (size_t i = 0; i < v1.size(); ++i)
        {
            result[i] = v1[i] + v2[i];
        }
        return result;
    }

} // namespace parafoil



#endif // MATH_UTIL_HPP