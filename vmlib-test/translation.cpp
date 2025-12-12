#include <catch2/catch_amalgamated.hpp>

#include "../vmlib/mat44.hpp"
#include "../vmlib/vec4.hpp"

using namespace Catch::Matchers;

TEST_CASE("make_translation", "[mat44][translation]")
{
    static constexpr float kEps_ = 1e-6f;

    SECTION("Zero translation ? Identity")
    {
        Mat44f T = make_translation({ 0.f, 0.f, 0.f });

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                REQUIRE_THAT((T[i, j]), WithinAbs(kIdentity44f[i, j], kEps_));
    }

    SECTION("Translate by (2, 3, 4)")
    {
        Mat44f T = make_translation({ 2.f, 3.f, 4.f });

        REQUIRE_THAT((T[0, 3]), WithinAbs(2.f, kEps_));
        REQUIRE_THAT((T[1, 3]), WithinAbs(3.f, kEps_));
        REQUIRE_THAT((T[2, 3]), WithinAbs(4.f, kEps_));

        Vec4f v{ 1.f, 1.f, 1.f, 1.f };
        Vec4f r = T * v;

        REQUIRE_THAT(r.x, WithinAbs(3.f, kEps_));
        REQUIRE_THAT(r.y, WithinAbs(4.f, kEps_));
        REQUIRE_THAT(r.z, WithinAbs(5.f, kEps_));
    }

    SECTION("Translate by negative offsets")
    {
        Mat44f T = make_translation({ -5.f, 10.f, -2.f });

        REQUIRE_THAT((T[0, 3]), WithinAbs(-5.f, kEps_));
        REQUIRE_THAT((T[1, 3]), WithinAbs(10.f, kEps_));
        REQUIRE_THAT((T[2, 3]), WithinAbs(-2.f, kEps_));
    }
}
