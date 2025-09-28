#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "rphys/api_world.h"
#include <cmath>

TEST_CASE("world_create_and_step_basic", "[world]") {
    rphys::world_desc desc{};
    auto wid = rphys::create_world(desc);
    REQUIRE(wid.value != 0);

    REQUIRE(rphys::world_frame_count(wid) == 0);
    REQUIRE(rphys::world_total_time(wid) == Catch::Approx(0.0));

    const int steps = 10;
    const double dt = 0.016;
    double accum = 0.0;
    for (int i = 0; i < steps; ++i) {
        rphys::step_world(wid, dt);
        accum += dt;
    }

    REQUIRE(rphys::world_frame_count(wid) == static_cast<std::uint64_t>(steps));
    REQUIRE(rphys::world_total_time(wid) == Catch::Approx(accum));

    // Negative dt: should increment frame but not advance time (clamped)
    double before_time = rphys::world_total_time(wid);
    auto before_frames = rphys::world_frame_count(wid);
    rphys::step_world(wid, -1.0);
    REQUIRE(rphys::world_frame_count(wid) == before_frames + 1);
    REQUIRE(rphys::world_total_time(wid) == Catch::Approx(before_time));

    rphys::destroy_world(wid);
}

TEST_CASE("world_invalid_id_queries", "[world]") {
    rphys::world_id invalid{999999};
    REQUIRE(rphys::world_frame_count(invalid) == 0ULL);
    REQUIRE(rphys::world_total_time(invalid) == Catch::Approx(0.0));
}

TEST_CASE("world_reuse_slot", "[world]") {
    rphys::world_desc desc{};
    auto w1 = rphys::create_world(desc);
    REQUIRE(w1.value != 0);
    rphys::step_world(w1, 0.01);
    rphys::destroy_world(w1);

    // Create again; may reuse slot, should start clean
    auto w2 = rphys::create_world(desc);
    REQUIRE(w2.value != 0);
    REQUIRE(rphys::world_frame_count(w2) == 0);
    REQUIRE(rphys::world_total_time(w2) == Catch::Approx(0.0));
    rphys::destroy_world(w2);
}
