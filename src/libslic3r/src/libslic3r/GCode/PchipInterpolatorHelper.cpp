#include "libslic3r/GCode/PchipInterpolatorHelper.hpp"
#include "Slic3r/Exception.hpp"

#include <cmath>
#include <algorithm>

namespace Slic3r {

PchipInterpolatorHelper::PchipInterpolatorHelper(const std::vector<double>& x, const std::vector<double>& y)
{
    this->set_data(x, y);
}

void PchipInterpolatorHelper::set_data(const std::vector<double>& x, const std::vector<double>& y)
{
    if (x.size() != y.size() || x.size() < 2) {
        throw Slic3r::InvalidArgument("Input vectors must have the same size and contain at least two points.");
    }
    m_x = x;
    m_y = y;
    this->sort_data();
    this->compute_pchip();
}

void PchipInterpolatorHelper::sort_data()
{
    std::vector<std::pair<double, double>> data;
    data.reserve(m_x.size());
    for (size_t i = 0; i < m_x.size(); ++i) {
        data.emplace_back(m_x[i], m_y[i]);
    }
    std::sort(data.begin(), data.end());

    for (size_t i = 0; i < data.size(); ++i) {
        m_x[i] = data[i].first;
        m_y[i] = data[i].second;
    }
}

void PchipInterpolatorHelper::compute_pchip()
{
    size_t n = m_x.size() - 1;
    m_h.resize(n);
    m_delta.resize(n);
    m_d.resize(n + 1);

    for (size_t i = 0; i < n; ++i) {
        m_h[i]     = this->h(i);
        m_delta[i] = this->delta(i);
    }

    m_d[0] = m_delta[0];
    m_d[n] = m_delta[n - 1];
    for (size_t i = 1; i < n; ++i) {
        if (m_delta[i - 1] * m_delta[i] > 0) {
            double w1 = 2 * m_h[i] + m_h[i - 1];
            double w2 = m_h[i] + 2 * m_h[i - 1];
            m_d[i]    = (w1 + w2) / (w1 / m_delta[i - 1] + w2 / m_delta[i]);
        } else {
            m_d[i] = 0.0;
        }
    }
}

double PchipInterpolatorHelper::interpolate(double xi) const
{
    if (xi <= m_x.front()) {
        return m_y.front();
    }
    if (xi >= m_x.back()) {
        return m_y.back();
    }

    auto it = std::lower_bound(m_x.begin(), m_x.end(), xi);
    size_t i = std::distance(m_x.begin(), it) - 1;

    double h_i = m_h[i];
    double t   = (xi - m_x[i]) / h_i;
    double t2  = t * t;
    double t3  = t2 * t;

    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;

    return h00 * m_y[i] + h10 * h_i * m_d[i] + h01 * m_y[i + 1] + h11 * h_i * m_d[i + 1];
}

} // namespace Slic3r
