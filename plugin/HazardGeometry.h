//
// HazardGeometry.h
//
// 2D geometry for hazard shapes (circles and heading-relative arcs). No EQ or Detour types, so this
// can be checked in isolation.
//
// Coordinates: all tests run on the navmesh x/z plane, which is (EQ X, EQ Y). A glm::vec2 here is
// (meshX, meshZ) == (EQ X, EQ Y). Height is handled by the caller.
//
// Heading convention: EQ headings run 0..512 for a full turn. theta = heading * pi / 256. A spawn's
// facing vector in (EQ X, EQ Y) is (sin theta, cos theta), matching MQ2's face code
// (atan2(dx, dy) * 256 / pi) and MQ2MoveUtils. The bearing of point P from centre C is
// atan2(P.x - C.x, P.y - C.y); relative angle 0 is in front of the spawn, pi is directly behind.
//

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

namespace hazard {

constexpr float kPi = glm::pi<float>();
constexpr float kTwoPi = 2.0f * glm::pi<float>();

inline float WrapAngle(float a)
{
	a = std::fmod(a + kPi, kTwoPi);
	if (a < 0) a += kTwoPi;
	return a - kPi; // [-pi, pi)
}

inline float HeadingToRadians(float eqHeading) { return eqHeading * kPi / 256.0f; }
inline float DegToRad(float deg) { return deg * kPi / 180.0f; }
inline float RadToDeg(float rad) { return rad * 180.0f / kPi; }

inline float Bearing(const glm::vec2& from, const glm::vec2& to)
{
	return std::atan2(to.x - from.x, to.y - from.y);
}

inline glm::vec2 DirectionFromAngle(float angle)
{
	return { std::sin(angle), std::cos(angle) };
}

// A circle, or a sector of a circle relative to the spawn's facing.
struct Shape
{
	glm::vec2 center{ 0.f };
	float facing = 0.f;            // radians, from the spawn heading
	float arcCenter = 0.f;         // radians, relative to facing (0 = front, pi = back)
	float arcHalfWidth = kPi;      // radians; >= pi means a full circle

	bool IsFullCircle() const { return arcHalfWidth >= kPi - 1e-4f; }

	// convex shapes let "all vertices inside" imply "whole polygon inside"
	bool IsConvex() const { return IsFullCircle() || arcHalfWidth <= kPi * 0.5f + 1e-4f; }

	float ArcMidAngle() const { return facing + arcCenter; }
	float ArcStartAngle() const { return ArcMidAngle() - arcHalfWidth; }
	float ArcEndAngle() const { return ArcMidAngle() + arcHalfWidth; }

	bool InAngularRange(const glm::vec2& p) const
	{
		if (IsFullCircle())
			return true;
		glm::vec2 d = p - center;
		if (glm::dot(d, d) < 1e-6f)
			return true; // the apex belongs to the sector
		return std::fabs(WrapAngle(Bearing(center, p) - ArcMidAngle())) <= arcHalfWidth;
	}
};

inline bool PointInShape(const Shape& s, const glm::vec2& p, float r)
{
	glm::vec2 d = p - s.center;
	return glm::dot(d, d) <= r * r && s.InAngularRange(p);
}

inline float DistancePointSegment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b)
{
	glm::vec2 ab = b - a;
	float len2 = glm::dot(ab, ab);
	float t = len2 > 1e-8f ? std::clamp(glm::dot(p - a, ab) / len2, 0.f, 1.f) : 0.f;
	return glm::length(p - (a + ab * t));
}

// Distance from p to the shape of radius r (0 if inside).
inline float DistanceToShape(const Shape& s, const glm::vec2& p, float r)
{
	if (PointInShape(s, p, r))
		return 0.f;

	float dist = glm::length(p - s.center);
	if (s.IsFullCircle())
		return std::max(0.f, dist - r);

	glm::vec2 edgeA = s.center + DirectionFromAngle(s.ArcStartAngle()) * r;
	glm::vec2 edgeB = s.center + DirectionFromAngle(s.ArcEndAngle()) * r;
	float best = std::min(DistancePointSegment(p, s.center, edgeA), DistancePointSegment(p, s.center, edgeB));
	if (s.InAngularRange(p))
		best = std::min(best, std::fabs(dist - r));
	return best;
}

inline bool SegmentsIntersect(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c, const glm::vec2& d)
{
	auto cross = [](const glm::vec2& u, const glm::vec2& v) { return u.x * v.y - u.y * v.x; };
	glm::vec2 r = b - a, q = d - c;
	float denom = cross(r, q);
	if (std::fabs(denom) < 1e-8f)
		return false; // parallel; endpoint containment is tested separately by callers
	float t = cross(c - a, q) / denom;
	float u = cross(c - a, r) / denom;
	return t >= 0.f && t <= 1.f && u >= 0.f && u <= 1.f;
}

// Does segment ab touch the shape of radius r?
inline bool SegmentIntersectsShape(const glm::vec2& a, const glm::vec2& b, const Shape& s, float r)
{
	if (s.IsFullCircle())
		return DistancePointSegment(s.center, a, b) <= r;

	if (PointInShape(s, a, r) || PointInShape(s, b, r))
		return true;

	// crosses either radial edge
	glm::vec2 edgeA = s.center + DirectionFromAngle(s.ArcStartAngle()) * r;
	glm::vec2 edgeB = s.center + DirectionFromAngle(s.ArcEndAngle()) * r;
	if (SegmentsIntersect(a, b, s.center, edgeA) || SegmentsIntersect(a, b, s.center, edgeB))
		return true;

	// crosses the arc: intersect with the circle, then check the angular range of the hit points
	glm::vec2 d = b - a, f = a - s.center;
	float A = glm::dot(d, d);
	if (A < 1e-8f)
		return false;
	float B = 2.f * glm::dot(f, d);
	float C = glm::dot(f, f) - r * r;
	float disc = B * B - 4.f * A * C;
	if (disc < 0.f)
		return false;
	float sq = std::sqrt(disc);
	for (float t : { (-B - sq) / (2.f * A), (-B + sq) / (2.f * A) })
	{
		if (t >= 0.f && t <= 1.f && s.InAngularRange(a + d * t))
			return true;
	}
	return false;
}

// Point in convex/concave polygon (2D). verts are (x, y) pairs.
inline bool PointInPolygon(const glm::vec2& p, const std::vector<glm::vec2>& poly)
{
	bool inside = false;
	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
	{
		const glm::vec2& vi = poly[i];
		const glm::vec2& vj = poly[j];
		if (((vi.y > p.y) != (vj.y > p.y))
			&& (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x))
		{
			inside = !inside;
		}
	}
	return inside;
}

// Minimum distance from p to the polygon (0 if p is inside).
inline float DistancePointPolygon(const glm::vec2& p, const std::vector<glm::vec2>& poly)
{
	if (poly.size() >= 3 && PointInPolygon(p, poly))
		return 0.f;
	float best = FLT_MAX;
	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
		best = std::min(best, DistancePointSegment(p, poly[j], poly[i]));
	return best;
}

// Classify a polygon against the shape of radius r.
// Returns true if any part of the polygon is inside the shape. fullyInside is set when the whole
// polygon is inside (only reported for convex shapes, where all-vertices-inside implies it).
inline bool PolygonOverlapsShape(const std::vector<glm::vec2>& poly, const Shape& s, float r, bool* fullyInside)
{
	bool anyIn = false, allIn = true;
	for (const glm::vec2& v : poly)
	{
		if (PointInShape(s, v, r)) anyIn = true;
		else allIn = false;
	}

	if (fullyInside)
		*fullyInside = allIn && s.IsConvex();

	if (anyIn)
		return true;

	// shape apex (centre) inside the polygon
	if (poly.size() >= 3 && PointInPolygon(s.center, poly))
		return true;

	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
	{
		if (SegmentIntersectsShape(poly[j], poly[i], s, r))
			return true;
	}
	return false;
}

// Points along the outline of the shape (arc plus radial edges for sectors), for drawing.
inline void OutlinePoints(const Shape& s, float r, int segments, std::vector<glm::vec2>& out)
{
	out.clear();
	if (s.IsFullCircle())
	{
		for (int i = 0; i <= segments; ++i)
			out.push_back(s.center + DirectionFromAngle(kTwoPi * i / segments) * r);
		return;
	}

	out.push_back(s.center);
	float start = s.ArcStartAngle();
	float span = 2.f * s.arcHalfWidth;
	int n = std::max(2, static_cast<int>(segments * span / kTwoPi));
	for (int i = 0; i <= n; ++i)
		out.push_back(s.center + DirectionFromAngle(start + span * i / n) * r);
	out.push_back(s.center);
}

} // namespace hazard
