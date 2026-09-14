#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"
#include "Slic3r/Biz/Algorithms/ModelObject.hpp"
#include "Slic3r/Domain/ModelVolume.hpp"
#include "Slic3r/Biz/GCodeReader/GCodeReader.hpp"

#include "test_data.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;
using namespace Catch;
using Domain::FloatOrPercentage;
using Domain::Percentage;

SCENARIO("PrintObject: object layer heights", "[PrintObject]") {
    GIVEN("20mm cube and default initial config, initial layer height of 2mm") {
        WHEN("generate_object_layers() is called for 2mm layer heights and nozzle diameter of 3mm") {
            Slic3r::Print print;

            TestConfig config{1, 3.0};
            config.print.items.opt("first_layer_height").set(FloatOrPercentage{2.0});
            config.print.items.opt("layer_height").set(2.0);

            Slic3r::Test::init_and_process_print({TestMesh::cube_20x20x20}, print, config);
            SpanOfConstPtrs<Layer> layers = print.objects().front()->layers();
            THEN("The output vector has 10 entries") {
                REQUIRE(layers.size() == 10);
            }
            AND_THEN("Each layer is approximately 2mm above the previous Z") {
                double last = 0.0;
                for (size_t i = 0; i < layers.size(); ++ i) {
                    REQUIRE((layers[i]->print_z - last) == Approx(2.0));
                    last = layers[i]->print_z;
                }
            }
        }
        WHEN("generate_object_layers() is called for 10mm layer heights and nozzle diameter of 11mm") {
            Slic3r::Print print;

            TestConfig config{1, 11.0};
            config.print.items.opt("first_layer_height").set(FloatOrPercentage{2.0});
            config.print.items.opt("layer_height").set(10.0);
            Slic3r::Test::init_and_process_print({TestMesh::cube_20x20x20}, print, config);

            SpanOfConstPtrs<Layer> layers = print.objects().front()->layers();
			THEN("The output vector has 3 entries") {
                REQUIRE(layers.size() == 3);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE(layers.front()->print_z == Approx(2.0));
            }
            AND_THEN("Layer 1 is at 12mm") {
                REQUIRE(layers[1]->print_z == Approx(12.0));
            }
        }
        WHEN("generate_object_layers() is called for 15mm layer heights and nozzle diameter of 16mm") {
            Slic3r::Print print;

            TestConfig config{1, 16.0};
            config.print.items.opt("first_layer_height").set(FloatOrPercentage{2.0});
            config.print.items.opt("layer_height").set(15.0);

            Slic3r::Test::init_and_process_print({TestMesh::cube_20x20x20}, print, config);
            SpanOfConstPtrs<Layer> layers = print.objects().front()->layers();

			THEN("The output vector has 2 entries") {
                REQUIRE(layers.size() == 2);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE(layers[0]->print_z == Approx(2.0));
            }
            AND_THEN("Layer 1 is at 17mm") {
                REQUIRE(layers[1]->print_z == Approx(17.0));
            }
        }
    }
}

TEST_CASE("PrintObject: extreme mesh coordinates (regression test for coord_t overflow)", "[PrintObject]") {
    using Slic3r::Biz::Algorithms::TriangleMesh::make_cube;

    GIVEN("A 20mm cube translated to Y=3000mm (beyond coord_t range when scaled)") {
        Domain::TriangleMesh cube = make_cube(20.0, 20.0, 20.0);

        const float offset{3000.0f};
        // Center will be at Y=3010mm, scaled = 3,010,000,000 > int32_t max (2,147,483,647)
        cube.translate(Vec3f(0.0f, offset, 0.0f));

        Print print;
        TestConfig config;
        Domain::Model model;
        config.print.items.opt("layer_height").set(0.2);

        Domain::ModelObject *object = model.add_object();
        object->name += "cube.stl";

        using Biz::Algorithms::ModelObject::add_volume;
        Domain::ModelVolume* volume{add_volume(object, cube)};
        object->add_instance();
        volume->set_offset(Vec3d{0.0, -offset, 0.0});

        init_print(std::initializer_list<Domain::TriangleMesh>{}, print, model, config, 0, true);
        REQUIRE_NOTHROW(print.process());

        THEN("Print has layers (slicing succeeded)") {
            REQUIRE(!print.objects().empty());
            const PrintObject* obj = print.objects().front();
            REQUIRE(obj->layers().size() > 0);
        }

        AND_THEN("Layers have non-empty slices") {
            const PrintObject* obj = print.objects().front();
            size_t non_empty_layers = 0;
            for (const Layer* layer : obj->layers()) {
                if (!layer->lslices.empty()) {
                    non_empty_layers++;
                }
            }

            REQUIRE(non_empty_layers == obj->layers().size());
        }

        AND_THEN("Instance shift is correct (no overflow)") {
            const PrintObject* obj = print.objects().front();
            const PrintInstance& inst = obj->instances().front();
            REQUIRE(inst.shift().y() > 0); // Should be positive, not negative from overflow.
        }
    }
}

TEST_CASE("PrintObject: per-volume and modifier extrusion multiplier", "[PrintObject][ExtrusionMultiplier]") {
    using Slic3r::Biz::Algorithms::TriangleMesh::make_cube;
    using Biz::Algorithms::ModelObject::add_volume;

    SECTION("Two solid volumes with different extrusion multipliers create separate PrintRegions and prevent merging") {
        Print print;
        TestConfig config;
        Domain::Model model;
        config.print.items.opt("layer_height").set(0.2);
        config.print.items.opt("first_layer_height").set(FloatOrPercentage{0.2});
        config.print.items.opt("first_layer_extrusion_width").set(FloatOrPercentage{0});
        config.print.items.opt("perimeters").set(1);
        config.print.items.opt("top_solid_layers").set(0);
        config.print.items.opt("bottom_solid_layers").set(0);
        config.print.items.opt("fill_density").set(Percentage{0});
        config.print.items.opt("skirts").set(0);
        config.printer.items.opt("start_gcode").set("");

        Domain::ModelObject *object = model.add_object();
        object->name += "two_cubes.stl";

        Domain::TriangleMesh cube1 = make_cube(10.0, 10.0, 10.0);
        Domain::TriangleMesh cube2 = make_cube(10.0, 10.0, 10.0);
        cube2.translate(Vec3f(25.0f, 0.0f, 0.0f));

        Domain::ModelVolume* vol1 = add_volume(object, cube1);
        Domain::ModelVolume* vol2 = add_volume(object, cube2);
        object->add_instance();

        vol1->volume_settings.overrides.set("object_extrusion_multiplier", 1.5);
        vol2->volume_settings.overrides.set("object_extrusion_multiplier", 0.5);

        init_print(std::initializer_list<Domain::TriangleMesh>{}, print, model, config, 0, true);
        REQUIRE_NOTHROW(print.process());

        REQUIRE(print.objects().size() == 1);
        const PrintObject* obj = print.objects().front();

        // 1. Two distinct PrintRegions should have been created
        REQUIRE(obj->shared_regions()->all_regions.size() == 2);
        std::vector<double> multipliers;
        for (const auto& region : obj->shared_regions()->all_regions) {
            multipliers.push_back(region->config().get<double>("object_extrusion_multiplier"));
        }
        std::sort(multipliers.begin(), multipliers.end());
        CHECK(multipliers[0] == Approx(0.5));
        CHECK(multipliers[1] == Approx(1.5));

        // 2. Layers should have 2 separate LayerRegions, neither empty, maintaining separation
        REQUIRE(!obj->layers().empty());
        for (const Layer* layer : obj->layers()) {
            REQUIRE(layer->regions().size() == 2);
            for (const LayerRegion* layerm : layer->regions()) {
                CHECK(!layerm->perimeters().empty());
                CHECK(!layerm->slices().empty());
            }
        }

        // 3. Exported G-code should reflect the overridden extrusion rates
        std::string gcode_str = Slic3r::Test::gcode(print);
        REQUIRE(!gcode_str.empty());

        Biz::GCodeReader::GCodeReader parser;
        std::vector<double> extrusion_rates;
        parser.parse_buffer(gcode_str, [&](Biz::GCodeReader::GCodeReader& self, const Biz::GCodeReader::GCodeReader::GCodeLine& line) {
            if (self.z() > 0.3f && line.extruding(self) && line.dist_XY(self) > 2.0f) {
                extrusion_rates.push_back(line.dist_E(self) / line.dist_XY(self));
            }
        });

        REQUIRE(!extrusion_rates.empty());
        double min_rate = *std::min_element(extrusion_rates.begin(), extrusion_rates.end());
        double max_rate = *std::max_element(extrusion_rates.begin(), extrusion_rates.end());
        // Multiplier 1.5 vs 0.5 means max/min extrusion rate ratio is 1.5 / 0.5 = 3.0
        CHECK(max_rate / min_rate == Approx(3.0).epsilon(0.05));
    }

    SECTION("Modifier volume overriding extrusion multiplier splits regions and configures flow") {
        Print print;
        TestConfig config;
        Domain::Model model;
        config.print.items.opt("layer_height").set(0.2);
        config.print.items.opt("first_layer_height").set(FloatOrPercentage{0.2});
        config.print.items.opt("first_layer_extrusion_width").set(FloatOrPercentage{0});
        config.print.items.opt("perimeters").set(1);
        config.print.items.opt("top_solid_layers").set(0);
        config.print.items.opt("bottom_solid_layers").set(0);
        config.print.items.opt("fill_density").set(Percentage{0});
        config.print.items.opt("skirts").set(0);
        config.printer.items.opt("start_gcode").set("");

        Domain::ModelObject *object = model.add_object();
        object->name += "cube_with_modifier.stl";

        // Base 20x20x10 cube
        Domain::TriangleMesh base_cube = make_cube(20.0, 20.0, 10.0);
        Domain::ModelVolume* base_vol = add_volume(object, base_cube);

        // Modifier 10x10x10 cube overlapping the center of the base cube
        Domain::TriangleMesh mod_cube = make_cube(10.0, 10.0, 10.0);
        mod_cube.translate(Vec3f(5.0f, 5.0f, 0.0f));
        Domain::ModelVolume* mod_vol = add_volume(object, mod_cube);
        mod_vol->set_type(Domain::ModelVolumeType::PARAMETER_MODIFIER);
        object->add_instance();

        // Base volume inherits default 1.0 (default flow); modifier volume overrides to 2.0
        mod_vol->volume_settings.overrides.set("object_extrusion_multiplier", 2.0);

        init_print(std::initializer_list<Domain::TriangleMesh>{}, print, model, config, 0, true);
        REQUIRE_NOTHROW(print.process());

        const PrintObject* obj = print.objects().front();
        REQUIRE(obj->shared_regions()->all_regions.size() == 2);

        std::vector<double> multipliers;
        for (const auto& region : obj->shared_regions()->all_regions) {
            multipliers.push_back(region->config().get<double>("object_extrusion_multiplier"));
        }
        std::sort(multipliers.begin(), multipliers.end());
        CHECK(multipliers[0] == Approx(1.0));
        CHECK(multipliers[1] == Approx(2.0));

        // Layers intersecting the modifier have 2 regions
        REQUIRE(!obj->layers().empty());
        for (const Layer* layer : obj->layers()) {
            REQUIRE(layer->regions().size() == 2);
            for (const LayerRegion* layerm : layer->regions()) {
                CHECK(!layerm->perimeters().empty());
            }
        }

        // Exported G-code should contain extrusions with ratio ~2.0
        std::string gcode_str = Slic3r::Test::gcode(print);
        REQUIRE(!gcode_str.empty());

        Biz::GCodeReader::GCodeReader parser;
        std::vector<double> extrusion_rates;
        parser.parse_buffer(gcode_str, [&](Biz::GCodeReader::GCodeReader& self, const Biz::GCodeReader::GCodeReader::GCodeLine& line) {
            if (self.z() > 0.3f && line.extruding(self) && line.dist_XY(self) > 2.0f) {
                extrusion_rates.push_back(line.dist_E(self) / line.dist_XY(self));
            }
        });

        REQUIRE(!extrusion_rates.empty());
        double min_rate = *std::min_element(extrusion_rates.begin(), extrusion_rates.end());
        double max_rate = *std::max_element(extrusion_rates.begin(), extrusion_rates.end());
        CHECK(max_rate / min_rate == Approx(2.0).epsilon(0.05));
    }

    SECTION("Object-level multiplier is inherited by volumes unless overridden per-volume") {
        Print print;
        TestConfig config;
        Domain::Model model;
        config.print.items.opt("layer_height").set(0.2);
        config.print.items.opt("first_layer_height").set(FloatOrPercentage{0.2});

        Domain::ModelObject *object = model.add_object();
        // Object sets multiplier = 1.6
        object->object_settings.overrides.set("object_extrusion_multiplier", 1.6);

        Domain::TriangleMesh cube1 = make_cube(10.0, 10.0, 10.0);
        Domain::TriangleMesh cube2 = make_cube(10.0, 10.0, 10.0);
        cube2.translate(Vec3f(25.0f, 0.0f, 0.0f));

        Domain::ModelVolume* vol1 = add_volume(object, cube1);
        Domain::ModelVolume* vol2 = add_volume(object, cube2);
        object->add_instance();

        // vol1 does not override -> inherits 1.6 from object
        // vol2 overrides to 0.8
        vol2->volume_settings.overrides.set("object_extrusion_multiplier", 0.8);

        init_print(std::initializer_list<Domain::TriangleMesh>{}, print, model, config, 0, true);
        REQUIRE_NOTHROW(print.process());

        const PrintObject* obj = print.objects().front();
        REQUIRE(obj->shared_regions()->all_regions.size() == 2);

        std::vector<double> multipliers;
        for (const auto& region : obj->shared_regions()->all_regions) {
            multipliers.push_back(region->config().get<double>("object_extrusion_multiplier"));
        }
        std::sort(multipliers.begin(), multipliers.end());
        CHECK(multipliers[0] == Approx(0.8));
        CHECK(multipliers[1] == Approx(1.6));
    }

    SECTION("Different height volumes preserve extrusion multiplier for upper internal infill") {
        Print print;
        TestConfig config;
        Domain::Model model;
        config.print.items.opt("layer_height").set(0.2);
        config.print.items.opt("first_layer_height").set(FloatOrPercentage{0.2});
        config.print.items.opt("fill_density").set(Percentage{20});
        config.print.items.opt("skirts").set(0);
        config.print.items.opt("gcode_comments").set(true);
        config.printer.items.opt("start_gcode").set("");

        Domain::ModelObject *object = model.add_object();
        object->name += "b_and_c_heights.stl";

        // B: 20x20x10 cube
        Domain::TriangleMesh cubeB = make_cube(20.0, 20.0, 10.0);
        // C: 20x20x20 cube (taller), offset in X
        Domain::TriangleMesh cubeC = make_cube(20.0, 20.0, 20.0);
        cubeC.translate(Vec3f(30.0f, 0.0f, 0.0f));

        Domain::ModelVolume* volB = add_volume(object, cubeB);
        Domain::ModelVolume* volC = add_volume(object, cubeC);
        object->add_instance();

        volB->volume_settings.overrides.set("object_extrusion_multiplier", 0.5);
        volC->volume_settings.overrides.set("object_extrusion_multiplier", 1.9);

        init_print(std::initializer_list<Domain::TriangleMesh>{}, print, model, config, 0, true);
        REQUIRE_NOTHROW(print.process());

        const PrintObject* obj = print.objects().front();
        REQUIRE(obj->layers().size() >= 100);

        // Check Layer 75 (at Z = 15.0mm, well above B which is 10mm tall)
        const Layer* layer15 = obj->get_layer_at_printz(15.0, 0.01);
        REQUIRE(layer15 != nullptr);

        std::string gcode_str = Slic3r::Test::gcode(print);
        REQUIRE(!gcode_str.empty());

        Biz::GCodeReader::GCodeReader parser;
        std::vector<double> lower_c_infill_rates;
        std::vector<double> upper_c_infill_rates;
        std::vector<double> lower_b_infill_rates;

        parser.parse_buffer(gcode_str, [&](Biz::GCodeReader::GCodeReader& self, const Biz::GCodeReader::GCodeReader::GCodeLine& line) {
            if (line.extruding(self) && line.dist_XY(self) > 2.0f) {
                if (line.comment().find("infill") != std::string_view::npos && line.comment().find("solid") == std::string_view::npos) {
                    double rate = line.dist_E(self) / line.dist_XY(self);
                    if (self.z() > 4.0f && self.z() < 8.0f) {
                        // At Z ~ 6mm, X < 25 is Cube B, X > 25 is Cube C
                        if (self.x() < 25.0f) {
                            lower_b_infill_rates.push_back(rate);
                        } else {
                            lower_c_infill_rates.push_back(rate);
                        }
                    } else if (self.z() > 12.0f && self.z() < 18.0f) {
                        // Only Cube C exists
                        upper_c_infill_rates.push_back(rate);
                    }
                }
            }
        });

        REQUIRE(!lower_c_infill_rates.empty());
        REQUIRE(!upper_c_infill_rates.empty());

        // Upper C infill rate should match lower C infill rate (both 1.9x multiplier)
        CHECK(upper_c_infill_rates.front() == Approx(lower_c_infill_rates.front()).epsilon(0.01));
    }

    SECTION("Volumetric speed cap scales down speed according to object_extrusion_multiplier") {
        Print print;
        TestConfig config;
        Domain::Model model;
        config.print.items.opt("layer_height").set(0.2);
        config.print.items.opt("first_layer_height").set(FloatOrPercentage{0.2});
        config.print.items.opt("perimeters").set(1);
        config.print.items.opt("top_solid_layers").set(0);
        config.print.items.opt("bottom_solid_layers").set(0);
        config.print.items.opt("fill_density").set(Percentage{0});
        config.print.items.opt("perimeter_speed").set(100.0);
        config.print.items.opt("external_perimeter_speed").set(FloatOrPercentage{100.0});
        config.print.items.opt("small_perimeter_speed").set(FloatOrPercentage{100.0});
        config.print.items.opt("max_volumetric_speed").set(1.0);
        config.print.items.opt("skirts").set(0);
        config.printer.items.opt("start_gcode").set("");

        Domain::ModelObject *object = model.add_object();
        Domain::TriangleMesh cube1 = make_cube(20.0, 20.0, 10.0);
        Domain::TriangleMesh cube2 = make_cube(20.0, 20.0, 10.0);
        cube2.translate(Vec3f(35.0f, 0.0f, 0.0f));

        Domain::ModelVolume* vol1 = add_volume(object, cube1);
        Domain::ModelVolume* vol2 = add_volume(object, cube2);
        object->add_instance();

        vol1->volume_settings.overrides.set("object_extrusion_multiplier", 1.0);
        vol2->volume_settings.overrides.set("object_extrusion_multiplier", 2.0);

        init_print(std::initializer_list<Domain::TriangleMesh>{}, print, model, config, 0, true);
        REQUIRE_NOTHROW(print.process());

        std::string gcode_str = Slic3r::Test::gcode(print);
        REQUIRE(!gcode_str.empty());

        Biz::GCodeReader::GCodeReader parser;
        std::vector<double> extrusion_speeds;

        parser.parse_buffer(gcode_str, [&](Biz::GCodeReader::GCodeReader& self, const Biz::GCodeReader::GCodeReader::GCodeLine& line) {
            if (self.z() > 0.3f && line.extruding(self) && line.dist_XY(self) > 2.0f) {
                const double feedrate = line.has_f() ? line.f() : self.f();
                if (feedrate > 0)
                    extrusion_speeds.push_back(feedrate);
            }
        });

        REQUIRE(!extrusion_speeds.empty());
        double min_speed = *std::min_element(extrusion_speeds.begin(), extrusion_speeds.end());
        double max_speed = *std::max_element(extrusion_speeds.begin(), extrusion_speeds.end());
        CHECK(max_speed / min_speed == Approx(2.0).epsilon(0.05));
    }
}
