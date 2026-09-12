#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "libslic3r/GCode/PchipInterpolatorHelper.hpp"
#include "libslic3r/GCode/SmallAreaInfillFlowCompensator.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "Slic3r/Exception.hpp"

using namespace Slic3r;

TEST_CASE("PCHIP interpolator evaluates endpoints and intermediate points correctly", "[SmallAreaFlowCompensation]") {
    std::vector<double> x = {0.0, 1.0, 2.0, 10.0};
    std::vector<double> y = {0.0, 0.5, 0.8, 1.0};

    PchipInterpolatorHelper interpolator(x, y);

    // Exact endpoints
    CHECK(interpolator.interpolate(0.0) == Catch::Approx(0.0));
    CHECK(interpolator.interpolate(10.0) == Catch::Approx(1.0));

    // Clamping outside domain
    CHECK(interpolator.interpolate(-1.0) == Catch::Approx(0.0));
    CHECK(interpolator.interpolate(20.0) == Catch::Approx(1.0));

    // Intermediate points monotonicity
    double v1 = interpolator.interpolate(0.5);
    double v2 = interpolator.interpolate(1.5);
    CHECK(v1 > 0.0);
    CHECK(v1 < 0.5);
    CHECK(v2 > 0.5);
    CHECK(v2 < 0.8);
}

TEST_CASE("SmallAreaInfillFlowCompensator parses model and modifies flow correctly", "[SmallAreaFlowCompensation]") {
    std::vector<std::string> model = {
        "0,0",
        "0.2,0.4444",
        "0.4,0.6145",
        "0.6,0.7059",
        "0.8,0.7619",
        "1.5,0.8571",
        "2,0.8889",
        "3,0.9231",
        "5,0.9520",
        "10,1"
    };

    SmallAreaInfillFlowCompensator compensator(model);

    // SolidInfill role flow reduction on short lines
    double dE_full = 1.0;
    double dE_short = compensator.modify_flow(0.5, dE_full, ExtrusionRole::SolidInfill);
    CHECK(dE_short < dE_full);
    CHECK(dE_short > 0.5 * dE_full);

    // TopSolidInfill role flow reduction
    double dE_top = compensator.modify_flow(0.5, dE_full, ExtrusionRole::TopSolidInfill);
    CHECK(dE_top == Catch::Approx(dE_short));

    // Lines beyond maximum model length are not reduced
    double dE_long = compensator.modify_flow(12.0, dE_full, ExtrusionRole::SolidInfill);
    CHECK(dE_long == Catch::Approx(dE_full));

    // Unsupported roles (e.g. Perimeter, InternalInfill) are unmodified
    double dE_perimeter = compensator.modify_flow(0.5, dE_full, ExtrusionRole::Perimeter);
    CHECK(dE_perimeter == Catch::Approx(dE_full));

    double dE_sparse = compensator.modify_flow(0.5, dE_full, ExtrusionRole::InternalInfill);
    CHECK(dE_sparse == Catch::Approx(dE_full));
}

TEST_CASE("SmallAreaInfillFlowCompensator validates model input constraints", "[SmallAreaFlowCompensation]") {
    // First length must be 0
    std::vector<std::string> invalid_first_len = {"1,0.5", "10,1"};
    CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(invalid_first_len), Slic3r::InvalidArgument);

    // Subsequent lengths must be increasing
    std::vector<std::string> invalid_decreasing_len = {"0,0", "5,0.5", "3,0.8", "10,1"};
    CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(invalid_decreasing_len), Slic3r::InvalidArgument);

    // Factors must be strictly increasing
    std::vector<std::string> invalid_decreasing_factors = {"0,0", "2,0.8", "5,0.7", "10,1"};
    CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(invalid_decreasing_factors), Slic3r::InvalidArgument);

    // Final factor must be 1.0
    std::vector<std::string> invalid_final_factor = {"0,0", "5,0.5", "10,0.9"};
    CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(invalid_final_factor), Slic3r::InvalidArgument);
}
