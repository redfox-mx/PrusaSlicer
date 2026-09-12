// Modify the flow of extrusion lines inversely proportional to the length of
// the extrusion line. When infill lines get shorter the flow rate will auto-
// matically be reduced to mitigate the effect of small infill areas being
// over-extruded.
//
// Based on original work by Alexander Þór licensed under the GPLv3:
// https://github.com/Alexander-T-Moss/Small-Area-Flow-Comp

#include "libslic3r/GCode/SmallAreaInfillFlowCompensator.hpp"
#include "libslic3r/libslic3r.h"
#include "Slic3r/Exception.hpp"
#include "Slic3r/Log.hpp"

#include <boost/algorithm/string/trim.hpp>

#include <cmath>
#include <sstream>
#include <limits>

namespace Slic3r {

static bool nearly_equal(double a, double b)
{
    return std::nextafter(a, std::numeric_limits<double>::lowest()) <= b && std::nextafter(a, std::numeric_limits<double>::max()) >= b;
}

SmallAreaInfillFlowCompensator::SmallAreaInfillFlowCompensator(const std::vector<std::string>& model_lines)
{
    try {
        for (const auto& line : model_lines) {
            std::istringstream iss(line);
            std::string        value_str;
            double             e_length = 0.0;

            if (std::getline(iss, value_str, ',')) {
                try {
                    boost::algorithm::trim(value_str);
                    if (value_str.empty()) {
                        continue;
                    }
                    e_length = std::stod(value_str);
                    if (std::getline(iss, value_str, ',')) {
                        boost::algorithm::trim(value_str);
                        m_extrusion_lengths.push_back(e_length);
                        m_flow_compensations.push_back(std::stod(value_str));
                    }
                } catch (...) {
                    std::stringstream ss;
                    ss << "Small Area Flow Compensation: Error parsing data point in small area infill compensation model: " << line;
                    throw Slic3r::InvalidArgument(ss.str());
                }
            }
        }

        for (size_t i = 0; i < m_extrusion_lengths.size(); ++i) {
            if (i == 0) {
                if (!nearly_equal(m_extrusion_lengths[i], 0.0)) {
                    throw Slic3r::InvalidArgument("Small Area Flow Compensation: First extrusion length for small area infill compensation model must be 0");
                }
            } else {
                if (nearly_equal(m_extrusion_lengths[i], 0.0)) {
                    throw Slic3r::InvalidArgument("Small Area Flow Compensation: Only the first extrusion length for small area infill compensation model can be 0");
                }
                if (m_extrusion_lengths[i] <= m_extrusion_lengths[i - 1]) {
                    throw Slic3r::InvalidArgument("Small Area Flow Compensation: Extrusion lengths for subsequent points must be increasing");
                }
            }
        }

        for (size_t i = 1; i < m_flow_compensations.size(); ++i) {
            if (m_flow_compensations[i] <= m_flow_compensations[i - 1]) {
                throw Slic3r::InvalidArgument("Small Area Flow Compensation: Flow compensation factors must strictly increase with extrusion length");
            }
        }

        if (!m_flow_compensations.empty() && !nearly_equal(m_flow_compensations.back(), 1.0)) {
            throw Slic3r::InvalidArgument("Small Area Flow Compensation: Final compensation factor for small area infill flow compensation model must be 1.0");
        }

        if (m_extrusion_lengths.size() >= 2) {
            m_flow_model = std::make_unique<PchipInterpolatorHelper>(m_extrusion_lengths, m_flow_compensations);
        }

    } catch (const std::exception& e) {
        SPDLOG_ERROR("Error parsing small area infill compensation model: {}", e.what());
        throw;
    }
}

SmallAreaInfillFlowCompensator::~SmallAreaInfillFlowCompensator() = default;

double SmallAreaInfillFlowCompensator::flow_comp_model(const double line_length) const
{
    if (!m_flow_model) {
        return 1.0;
    }

    if (line_length <= 0.0 || line_length >= this->max_modified_length()) {
        return 1.0;
    }

    return m_flow_model->interpolate(line_length);
}

double SmallAreaInfillFlowCompensator::modify_flow(const double line_length, const double dE, const ExtrusionRole role) const
{
    if (m_flow_model &&
        (role == ExtrusionRole::SolidInfill || role == ExtrusionRole::TopSolidInfill)) {
        return dE * this->flow_comp_model(line_length);
    }

    return dE;
}

} // namespace Slic3r
