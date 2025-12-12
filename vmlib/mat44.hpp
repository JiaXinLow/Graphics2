#ifndef MAT44_HPP_E7187A26_469E_48AD_A3D2_63150F05A4CA
#define MAT44_HPP_E7187A26_469E_48AD_A3D2_63150F05A4CA
// SOLUTION_TAGS: gl-(ex-[^12]|cw-2|resit)

#include <cmath>
#include <cassert>
#include <cstdlib>

#include "vec3.hpp"
#include "vec4.hpp"

/** Mat44f: 4x4 matrix with floats
 *
 * See vec2f.hpp for discussion. Similar to the implementation, the Mat44f is
 * intentionally kept simple and somewhat bare bones.
 *
 * The matrix is stored in row-major order (careful when passing it to OpenGL).
 *
 * The overloaded operator [] allows access to individual elements. Example:
 *    Mat44f m = ...;
 *    float m12 = m[1,2];
 *    m[0,3] = 3.f;
 *
 * (Multi-dimensionsal subscripts in operator[] is a C++23 feature!)
 *
 * The matrix is arranged as:
 *
 *   ⎛ 0,0  0,1  0,2  0,3 ⎞
 *   ⎜ 1,0  1,1  1,2  1,3 ⎟
 *   ⎜ 2,0  2,1  2,2  2,3 ⎟
 *   ⎝ 3,0  3,1  3,2  3,3 ⎠
 */
struct Mat44f
{
	float v[16];

	constexpr
	float& operator[] (std::size_t aI, std::size_t aJ) noexcept
	{
		assert( aI < 4 && aJ < 4 );
		return v[aI*4 + aJ];
	}
	constexpr
	float const& operator[] (std::size_t aI, std::size_t aJ) const noexcept
	{
		assert( aI < 4 && aJ < 4 );
		return v[aI*4 + aJ];
	}
};

// Identity matrix
constexpr Mat44f kIdentity44f = { {
	1.f, 0.f, 0.f, 0.f,
	0.f, 1.f, 0.f, 0.f,
	0.f, 0.f, 1.f, 0.f,
	0.f, 0.f, 0.f, 1.f
} };

// Common operators for Mat44f.
// Note that you will need to implement these yourself.

constexpr
Mat44f operator*( Mat44f const& aLeft, Mat44f const& aRight ) noexcept
{
	//TODO: your implementation goes here
	Mat44f R{}; // Result matrix

	for (int i = 0; i < 4; ++i) // Row index
	{
		for (int j = 0; j < 4; ++j) // Column index
		{
			float sum = 0.f;
			for (int k = 0; k < 4; ++k) // Dot product of row i and col j
				sum += aLeft[i, k] * aRight[k, j];
			R[i, j] = sum; // Assign element (i,j)
		}
	}
	return R; // Return multiplied matrix
}

constexpr
Vec4f operator*( Mat44f const& aLeft, Vec4f const& aRight ) noexcept
{
	//TODO: your implementation goes here
	// Multiply 4x4 matrix by 4D vector
	return {
		// Row 0 dot product with vector
		aLeft[0,0] * aRight.x + aLeft[0,1] * aRight.y + aLeft[0,2] * aRight.z + aLeft[0,3] * aRight.w,
		// Row 1 dot product with vector
		aLeft[1,0] * aRight.x + aLeft[1,1] * aRight.y + aLeft[1,2] * aRight.z + aLeft[1,3] * aRight.w,
		// Row 2 dot product with vector
		aLeft[2,0] * aRight.x + aLeft[2,1] * aRight.y + aLeft[2,2] * aRight.z + aLeft[2,3] * aRight.w,
		// Row 3 dot product with vector
		aLeft[3,0] * aRight.x + aLeft[3,1] * aRight.y + aLeft[3,2] * aRight.z + aLeft[3,3] * aRight.w
	};
}

// Functions:

Mat44f invert( Mat44f const& aM ) noexcept;

inline
Mat44f transpose( Mat44f const& aM ) noexcept
{
	Mat44f ret;
	for( std::size_t i = 0; i < 4; ++i )
	{
		for( std::size_t j = 0; j < 4; ++j )
			ret[j,i] = aM[i,j];
	}
	return ret;
}

inline
Mat44f make_rotation_x( float aAngle ) noexcept
{
	//TODO: your implementation goes here
	float c = std::cos(aAngle);
	float s = std::sin(aAngle);

	// Rotation around X axis
	return { {
		1, 0, 0, 0, // X axis unchanged
		0, c,-s, 0, // rotate Y,Z
		0, s, c, 0, // rotate Y,Z
		0, 0, 0, 1  // homogeneous coordinate
	} };
}

inline
Mat44f make_rotation_y( float aAngle ) noexcept
{
	//TODO: your implementation goes here
	float c = std::cos(aAngle);
	float s = std::sin(aAngle);

	// Rotation around Y axis
	return { {
		 c, 0, s, 0, // X,Z rotated
		 0, 1, 0, 0, // Y axis unchanged
		-s, 0, c, 0, // X,Z rotated
		 0, 0, 0, 1 // homogeneous coordinate
	} };
}

inline
Mat44f make_rotation_z( float aAngle ) noexcept
{
	//TODO: your implementation goes here
	float c = std::cos(aAngle);
	float s = std::sin(aAngle);

	// Rotation around Z axis
	return { {
		c,-s, 0, 0, // rotate X,Y
		s, c, 0, 0, // rotate X,Y
		0, 0, 1, 0, // Z axis unchanged
		0, 0, 0, 1 // homogeneous coordinate
	} };
}

inline
Mat44f make_translation( Vec3f aTranslation ) noexcept
{
	//TODO: your implementation goes here
	// Build 4x4 translation matrix
	return { {
		1, 0, 0, aTranslation.x, // move along X
		0, 1, 0, aTranslation.y, // move along Y
		0, 0, 1, aTranslation.z, // move along Z
		0, 0, 0, 1 // homogeneous coordinate
	} };
}

inline
Mat44f make_scaling( float aSX, float aSY, float aSZ ) noexcept
{
	//TODO: your implementation goes here
	// Build 4x4 scaling matrix
	return { {
		aSX, 0,  0,  0, // scale X
		0, aSY,  0,  0, // scale Y
		0,  0, aSZ, 0, // scale Z
		0,  0,  0,  1 // homogeneous coordinate
	} };
}

inline
Mat44f make_perspective_projection( float aFovInRadians, float aAspect, float aNear, float aFar ) noexcept
{
	//TODO: your implementation goes here
	// Compute scale factor from field of view
	float f = 1.0f / std::tan(aFovInRadians * 0.5f);
	// Reciprocal of near–far difference
	float nf = 1.0f / (aNear - aFar);

	// Build 4x4 perspective projection matrix
	return { {
		f / aAspect, 0, 0,                                0,  // scale X by aspect
		0,          f, 0,                                0,  // scale Y
		0,          0, (aFar + aNear) * nf,  2 * aFar * aNear * nf, // depth mapping
		0,          0, -1,                               0   // perspective divide
	} };
}

#endif // MAT44_HPP_E7187A26_469E_48AD_A3D2_63150F05A4CA