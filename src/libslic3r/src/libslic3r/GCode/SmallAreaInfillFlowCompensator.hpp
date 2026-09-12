#ifndef slic3r_GCode_SmallAreaInfillFlowCompensator_hpp_
#define slic3r_GCode_SmallAreaInfillFlowCompensator_hpp_

#include "libslic3r/libslic3r.h"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/GCode/PchipInterpolatorHelper.hpp"

#include <memory>
#include <vector>
#include <string>

namespace Slic3r {

/**
 * @brief Compensates flow for small infill areas based on extrusion line length.
 */
class SmallAreaInfillFlowCompensator
{
public:
    SmallAreaInfillFlowCompensator() = delete;

    /**
     * @brief Constructs compensator from configuration model lines.
     * @param model_lines Vector of strings representing comma-separated length,factor pairs.
     */
    explicit SmallAreaInfillFlowCompensator(const std::vector<std::string>& model_lines);
    ~SmallAreaInfillFlowCompensator();

    /**
     * @brief Modifies extrusion amount dE based on line length and extrusion role.
     * @param line_length Extrusion line length in mm.
     * @param dE Original extrusion amount.
     * @param role Extrusion role.
     * @return Adjusted extrusion amount.
     */
    double modify_flow(double line_length, double dE, ExtrusionRole role) const;

private:
    std::vector<double>                      m_extrusion_lengths;
    std::vector<double>                      m_flow_compensations;
    std::unique_ptr<PchipInterpolatorHelper> m_flow_model;

    double flow_comp_model(double line_length) const;

    double max_modified_length() const
    {
        return m_extrusion_lengths.empty() ? 0.0 : m_extrusion_lengths.back();
    }
};

} // namespace Slic3r

#endif // slic3r_GCode_SmallAreaInfillFlowCompensator_hpp_
