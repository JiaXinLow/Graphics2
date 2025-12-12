#include <catch2/catch_amalgamated.hpp>

#include "../vmlib/mat44.hpp"
#include "../vmlib/vec4.hpp"

using namespace Catch::Matchers;

static constexpr float kEps_ = 1e-5f;

TEST_CASE("Mat44f Matrix × Matrix", "[mat44][mult]")
{
    SECTION("Identity × Identity = Identity")
    {
        Mat44f A = kIdentity44f;
        Mat44f B = kIdentity44f;
        Mat44f R = A * B;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                REQUIRE_THAT((R[i, j]), WithinAbs(kIdentity44f[i, j], kEps_));
    }

    SECTION("Identity × M = M")
    {
        Mat44f M{ {
            1,2,3,4,
            5,6,7,8,
            9,10,11,12,
            13,14,15,16
        } };

        Mat44f R = kIdentity44f * M;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                REQUIRE_THAT((R[i, j]), WithinAbs(M[i, j], kEps_));
    }

    SECTION("General matrix multiplication (NumPy verified)")
    {
        Mat44f M1{ {
            1,2,3,4,
            4,3,2,1,
            1,0,1,0,
            0,1,0,1
        } };
        Mat44f M2{ {
            2,0,1,0,
            1,2,0,1,
            3,1,2,1,
            0,1,1,0
        } };
        Mat44f expected{ {
            13,11,11,5,
            17, 9, 9,5,
            5, 1, 3,1,
            1, 3, 1,1
        } };

        Mat44f R = M1 * M2;

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                REQUIRE_THAT((R[i, j]), WithinAbs(expected[i, j], kEps_));
    }
}

TEST_CASE("Mat44f Matrix × Vec4f", "[mat44][mult]")
{
    SECTION("Simple multiplication")
    {
        Mat44f M{ {
            1,2,3,4,
            5,6,7,8,
            9,10,11,12,
            13,14,15,16
        } };

        Vec4f v{ 1,2,3,1 };
        Vec4f r = M * v;

        REQUIRE_THAT(r.x, WithinAbs(18.f, kEps_));
        REQUIRE_THAT(r.y, WithinAbs(46.f, kEps_));
        REQUIRE_THAT(r.z, WithinAbs(74.f, kEps_));
        REQUIRE_THAT(r.w, WithinAbs(102.f, kEps_));
    }
}

TEST_CASE("make_scaling", "[mat44][scaling]")
{
    SECTION("Vector scaling")
    {
        Mat44f S = make_scaling(2.f, 3.f, 4.f);

        REQUIRE_THAT((S[0, 0]), WithinAbs(2.f, kEps_));
        REQUIRE_THAT((S[1, 1]), WithinAbs(3.f, kEps_));
        REQUIRE_THAT((S[2, 2]), WithinAbs(4.f, kEps_));

        Vec4f v{ 1,2,3,1 };
        Vec4f r = S * v;

        REQUIRE_THAT(r.x, WithinAbs(2.f, kEps_));
        REQUIRE_THAT(r.y, WithinAbs(6.f, kEps_));
        REQUIRE_THAT(r.z, WithinAbs(12.f, kEps_));
    }
}
