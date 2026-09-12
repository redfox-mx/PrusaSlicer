#ifndef slic3r_GCode_PchipInterpolatorHelper_hpp_
#define slic3r_GCode_PchipInterpolatorHelper_hpp_

#include <vector>
#include <cstddef>

namespace Slic3r {

/**
 * @brief A helper class to perform Piecewise Cubic Hermite Interpolating Polynomial (PCHIP) interpolation.
 */
class PchipInterpolatorHelper
{
public:
    /**
     * @brief Default constructor.
     */
    PchipInterpolatorHelper() = default;

    /**
     * @brief Constructs the PCHIP interpolator with given data points.
     * @param x The x-coordinates of the data points.
     * @param y The y-coordinates of the data points.
     */
    PchipInterpolatorHelper(const std::vector<double>& x, const std::vector<double>& y);

    /**
     * @brief Sets the data points for the interpolator.
     * @param x The x-coordinates of the data points.
     * @param y The y-coordinates of the data points.
     */
    void set_data(const std::vector<double>& x, const std::vector<double>& y);

    /**
     * @brief Interpolates the value at a given point.
     * @param xi The x-coordinate at which to interpolate.
     * @return The interpolated y-coordinate.
     */
    double interpolate(double xi) const;

private:
    std::vector<double> m_x;
    std::vector<double> m_y;
    std::vector<double> m_h;
    std::vector<double> m_delta;
    std::vector<double> m_d;

    /**
     * @brief Computes the PCHIP coefficients.
     */
    void compute_pchip();

    /**
     * @brief Sorts the data points by x-coordinate.
     */
    void sort_data();

    /**
     * @brief Computes the difference between successive x-coordinates.
     * @param i The index of the x-coordinate.
     * @return The difference between m_x[i+1] and m_x[i].
     */
    double h(size_t i) const
    {
        return m_x[i + 1] - m_x[i];
    }

    /**
     * @brief Computes the slope of the segment between successive data points.
     * @param i The index of the segment.
     * @return The slope of the segment between m_y[i] and m_y[i+1].
     */
    double delta(size_t i) const
    {
        return (m_y[i + 1] - m_y[i]) / h(i);
    }
};

} // namespace Slic3r

#endif // slic3r_GCode_PchipInterpolatorHelper_hpp_
