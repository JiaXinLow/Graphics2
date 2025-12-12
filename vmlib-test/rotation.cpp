#include <catch2/catch_amalgamated.hpp>
#include <numbers>
#include "../vmlib/mat44.hpp"
#include "../vmlib/vec4.hpp"

using namespace Catch::Matchers;

TEST_CASE("Rotation matrices", "[mat44][rotation]")
{
    static constexpr float kEps_ = 1e-6f;
    float angle = std::numbers::pi_v<float> / 2.f;   // 90 degrees

    SECTION("Rotation X (90°)")
    {
        Mat44f R = make_rotation_x(angle);

        Mat44f expected{ {
            1, 0, 0, 0,
            0, 0,-1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1
        } };

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                REQUIRE_THAT((R[i, j]), WithinAbs(expected[i, j], kEps_));

        // Vector test
        Vec4f v{ 0,1,0,1 };
        Vec4f r = R * v;
        REQUIRE_THAT(r.z, WithinAbs(1.f, kEps_));
    }

    SECTION("Rotation Y (90°)")
    {
        Mat44f R = make_rotation_y(angle);

        Mat44f expected{ {
             0, 0, 1, 0,
             0, 1, 0, 0,
            -1, 0, 0, 0,
             0, 0, 0, 1
        } };

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                REQUIRE_THAT((R[i, j]), WithinAbs(expected[i, j], kEps_));
    }

    SECTION("Rotation Z (90°)")
    {
        Mat44f R = make_rotation_z(angle);

        Mat44f expected{ {
            0,-1, 0, 0,
            1, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        } };

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                REQUIRE_THAT((R[i, j]), WithinAbs(expected[i, j], kEps_));
    }
}
