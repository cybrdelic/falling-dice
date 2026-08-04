#include "sim.hpp"

#include <deque>
#include <set>

namespace tower {
namespace {

struct PairCandidate {
    int a = -1;
    int b = -1;
    double distance2 = INF;
};

enum class SatKind : uint8_t { FaceA, FaceB, EdgeEdge };

struct SatResult {
    bool overlap = false;
    Vec3 normal{1, 0, 0};
    double penetration = 0;
    double separation = INF;
    SatKind kind = SatKind::FaceA;
    int axisA = 0;
    int axisB = 0;
};

struct MagneticFace {
    Vec3 center{0, 0, 0};
    Vec3 normal{1, 0, 0};
    Vec3 tangentU{0, 1, 0};
    Vec3 tangentV{0, 0, 1};
    std::array<Vec3, 4> patchPosition{};
    std::array<double, 4> patchCharge{};
};

struct MagneticGeometry {
    std::array<MagneticFace, 6> faces{};
};

struct Interaction {
    double potential = 0;
    Vec3 forceOnB{0, 0, 0};
    Vec3 torqueA{0, 0, 0};
    Vec3 torqueB{0, 0, 0};
    double nearestOpposite = INF;
    Vec3 nearestPointA{0, 0, 0};
    Vec3 nearestPointB{0, 0, 0};
    Vec3 captureNormal{1, 0, 0};
    double captureQuality = 0;
    int faceIndexA = -1;
    int faceIndexB = -1;
    bool facePairCaptured = false;
};

struct SmoothValue {
    double value = 0;
    double derivative = 0;
};


std::array<Vec3, 3> boxAxes(const Body& body) {
    return {
        normalize(body.q.rotate({1, 0, 0})),
        normalize(body.q.rotate({0, 1, 0})),
        normalize(body.q.rotate({0, 0, 1}))
    };
}

std::array<double, 3> coreHalf(const Body& body) {
    return {
        std::max(0.0001, body.half.x - body.rounding),
        std::max(0.0001, body.half.y - body.rounding),
        std::max(0.0001, body.half.z - body.rounding)
    };
}

Vec3 worldInvI(const Body& body, const Vec3& worldVector) {
    return body.q.rotate(body.invILocal * body.q.inverseRotate(worldVector));
}

void wake(Body& body) {
    body.sleeping = false;
    body.sleepTimer = 0;
}

Vec3 pointVelocity(const Body& body, const Vec3& point) {
    return body.v + cross(body.w, point - body.p);
}

void applyForceAt(Body& body, const Vec3& force, const Vec3& point) {
    if (body.invMass <= 0) return;
    body.force += force;
    body.torque += cross(point - body.p, force);
}

void applyImpulse(Body& body,
                  const Vec3& impulse,
                  const Vec3& point,
                  double /*wakeThreshold*/) {
    if (body.invMass <= 0 || body.sleeping) return;
    body.v += body.invMass * impulse;
    body.w += worldInvI(body, cross(point - body.p, impulse));
}

SmoothValue smootherstepWithDerivative(double a, double b, double x) {
    if (x <= a) return {0, 0};
    if (x >= b) return {1, 0};
    const double invRange = 1.0 / (b - a);
    const double t = (x - a) * invRange;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double value = t3 * (t * (t * 6.0 - 15.0) + 10.0);
    const double derivative = 30.0 * t2 * sq(t - 1.0) * invRange;
    return {value, derivative};
}

Vec3 supportRoundedBox(const Body& body, const Vec3& direction) {
    const auto axes = boxAxes(body);
    const auto core = coreHalf(body);
    Vec3 point = body.p;
    for (int axis = 0; axis < 3; ++axis) {
        const double projection = dot(direction, axes[size_t(axis)]);
        point += (projection >= 0 ? core[size_t(axis)] : -core[size_t(axis)]) *
            axes[size_t(axis)];
    }
    const double d2 = lengthSquared(direction);
    if (d2 > 1e-20) point += body.rounding * direction / std::sqrt(d2);
    return point;
}

std::array<Vec3, 8> coreCorners(const Body& body) {
    std::array<Vec3, 8> corners{};
    const auto axes = boxAxes(body);
    const auto core = coreHalf(body);
    int index = 0;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int sz : {-1, 1}) {
                corners[size_t(index++)] = body.p +
                    double(sx) * core[0] * axes[0] +
                    double(sy) * core[1] * axes[1] +
                    double(sz) * core[2] * axes[2];
            }
        }
    }
    return corners;
}

SatResult queryRoundedBoxes(const Body& a, const Body& b) {
    const auto axesA = boxAxes(a);
    const auto axesB = boxAxes(b);
    const auto coreA = coreHalf(a);
    const auto coreB = coreHalf(b);
    const Vec3 delta = b.p - a.p;

    double maximumSeparation = -INF;
    Vec3 separationAxis{1, 0, 0};
    SatKind separationKind = SatKind::FaceA;
    int separationAxisA = 0;
    int separationAxisB = 0;

    double minimumPenetration = INF;
    double minimumPenetrationScore = INF;
    Vec3 penetrationAxis{1, 0, 0};
    SatKind penetrationKind = SatKind::FaceA;
    int penetrationAxisA = 0;
    int penetrationAxisB = 0;

    auto testAxis = [&](Vec3 axis, SatKind kind, int indexA, int indexB) {
        const double axisLength2 = lengthSquared(axis);
        if (axisLength2 < 1e-16) return;
        axis /= std::sqrt(axisLength2);
        double radiusA = a.rounding;
        double radiusB = b.rounding;
        for (int k = 0; k < 3; ++k) {
            radiusA += coreA[size_t(k)] * std::abs(dot(axis, axesA[size_t(k)]));
            radiusB += coreB[size_t(k)] * std::abs(dot(axis, axesB[size_t(k)]));
        }
        const double signedDistance = dot(delta, axis);
        const double separation = std::abs(signedDistance) - radiusA - radiusB;
        const Vec3 orientedAxis = signedDistance >= 0 ? axis : -axis;
        if (separation > maximumSeparation) {
            maximumSeparation = separation;
            separationAxis = orientedAxis;
            separationKind = kind;
            separationAxisA = indexA;
            separationAxisB = indexB;
        }
        const double penetration = -separation;
        const double score = penetration *
            (kind == SatKind::EdgeEdge ? 1.08 : 1.0);
        if (score < minimumPenetrationScore) {
            minimumPenetrationScore = score;
            minimumPenetration = penetration;
            penetrationAxis = orientedAxis;
            penetrationKind = kind;
            penetrationAxisA = indexA;
            penetrationAxisB = indexB;
        }
    };

    for (int i = 0; i < 3; ++i) testAxis(axesA[size_t(i)], SatKind::FaceA, i, -1);
    for (int j = 0; j < 3; ++j) testAxis(axesB[size_t(j)], SatKind::FaceB, -1, j);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 edgeAxis = cross(axesA[size_t(i)], axesB[size_t(j)]);
            const double edgeLength2 = lengthSquared(edgeAxis);
            if (edgeLength2 < 1e-16) continue;
            edgeAxis /= std::sqrt(edgeLength2);
            double faceAlignment = 0;
            for (int k = 0; k < 3; ++k) {
                faceAlignment = std::max(faceAlignment,
                    std::abs(dot(edgeAxis, axesA[size_t(k)])));
                faceAlignment = std::max(faceAlignment,
                    std::abs(dot(edgeAxis, axesB[size_t(k)])));
            }
            // A cross product that duplicates a face normal is not an
            // edge-edge contact. Classifying it as one collapses a broad face
            // manifold to one hinge point and injects the curling motion that
            // was visible in the previous stack.
            if (faceAlignment > 0.985) continue;
            testAxis(edgeAxis, SatKind::EdgeEdge, i, j);
        }
    }

    SatResult result;
    if (maximumSeparation > 0) {
        result.overlap = false;
        result.normal = separationAxis;
        result.separation = maximumSeparation;
        result.penetration = -maximumSeparation;
        result.kind = separationKind;
        result.axisA = separationAxisA;
        result.axisB = separationAxisB;
    } else {
        result.overlap = true;
        result.normal = penetrationAxis;
        result.separation = -minimumPenetration;
        result.penetration = minimumPenetration;
        result.kind = penetrationKind;
        result.axisA = penetrationAxisA;
        result.axisB = penetrationAxisB;
    }
    return result;
}

std::vector<Vec3> faceVertices(const Body& body, int axisIndex, int faceSign) {
    const auto axes = boxAxes(body);
    const auto core = coreHalf(body);
    const int uIndex = (axisIndex + 1) % 3;
    const int vIndex = (axisIndex + 2) % 3;
    const Vec3 center = body.p +
        double(faceSign) * core[size_t(axisIndex)] * axes[size_t(axisIndex)];
    std::vector<Vec3> vertices;
    vertices.reserve(4);
    vertices.push_back(center - core[size_t(uIndex)] * axes[size_t(uIndex)] -
        core[size_t(vIndex)] * axes[size_t(vIndex)]);
    vertices.push_back(center + core[size_t(uIndex)] * axes[size_t(uIndex)] -
        core[size_t(vIndex)] * axes[size_t(vIndex)]);
    vertices.push_back(center + core[size_t(uIndex)] * axes[size_t(uIndex)] +
        core[size_t(vIndex)] * axes[size_t(vIndex)]);
    vertices.push_back(center - core[size_t(uIndex)] * axes[size_t(uIndex)] +
        core[size_t(vIndex)] * axes[size_t(vIndex)]);
    return vertices;
}

std::vector<Vec3> clipPolygon(const std::vector<Vec3>& polygon,
                              const Vec3& normal,
                              double offset) {
    std::vector<Vec3> output;
    if (polygon.empty()) return output;
    output.reserve(polygon.size() + 2);
    Vec3 previous = polygon.back();
    double previousDistance = dot(normal, previous) - offset;
    bool previousInside = previousDistance <= 1e-9;
    for (const Vec3& current : polygon) {
        const double currentDistance = dot(normal, current) - offset;
        const bool currentInside = currentDistance <= 1e-9;
        if (currentInside != previousInside) {
            const double denominator = previousDistance - currentDistance;
            const double t = std::abs(denominator) > 1e-15
                ? previousDistance / denominator
                : 0.0;
            output.push_back(mix(previous, current, saturate(t)));
        }
        if (currentInside) output.push_back(current);
        previous = current;
        previousDistance = currentDistance;
        previousInside = currentInside;
    }
    return output;
}

std::pair<Vec3, Vec3> edgeSegment(const Body& body,
                                  int edgeAxis,
                                  const Vec3& facingDirection) {
    const auto axes = boxAxes(body);
    const auto core = coreHalf(body);
    Vec3 center = body.p;
    for (int k = 0; k < 3; ++k) {
        if (k == edgeAxis) continue;
        const double sign = dot(facingDirection, axes[size_t(k)]) >= 0 ? 1.0 : -1.0;
        center += sign * core[size_t(k)] * axes[size_t(k)];
    }
    return {
        center - core[size_t(edgeAxis)] * axes[size_t(edgeAxis)],
        center + core[size_t(edgeAxis)] * axes[size_t(edgeAxis)]
    };
}

std::pair<Vec3, Vec3> closestSegmentPoints(const Vec3& p1,
                                           const Vec3& q1,
                                           const Vec3& p2,
                                           const Vec3& q2) {
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);
    double s = 0;
    double t = 0;
    if (a <= 1e-16 && e <= 1e-16) return {p1, p2};
    if (a <= 1e-16) {
        t = clamp(f / e, 0.0, 1.0);
    } else {
        const double c = dot(d1, r);
        if (e <= 1e-16) {
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = dot(d1, d2);
            const double denominator = a * e - b * b;
            if (std::abs(denominator) > 1e-16) {
                s = clamp((b * f - c * e) / denominator, 0.0, 1.0);
            }
            t = (b * s + f) / e;
            if (t < 0) {
                t = 0;
                s = clamp(-c / a, 0.0, 1.0);
            } else if (t > 1) {
                t = 1;
                s = clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return {p1 + s * d1, p2 + t * d2};
}

bool buildFaceContacts(const Body& reference,
                       const Body& incident,
                       int referenceAxis,
                       const Vec3& referenceToIncidentNormal,
                       double speculativeMargin,
                       std::vector<std::pair<Vec3, double>>& contacts) {
    const auto refAxes = boxAxes(reference);
    const auto incAxes = boxAxes(incident);
    const auto refCore = coreHalf(reference);
    const int refSign = dot(referenceToIncidentNormal,
                            refAxes[size_t(referenceAxis)]) >= 0 ? 1 : -1;
    const Vec3 refNormal = double(refSign) * refAxes[size_t(referenceAxis)];
    const Vec3 refCenter = reference.p +
        double(refSign) * refCore[size_t(referenceAxis)] *
        refAxes[size_t(referenceAxis)];
    const int refUIndex = (referenceAxis + 1) % 3;
    const int refVIndex = (referenceAxis + 2) % 3;
    const Vec3 refU = refAxes[size_t(refUIndex)];
    const Vec3 refV = refAxes[size_t(refVIndex)];
    const double refUExtent = refCore[size_t(refUIndex)];
    const double refVExtent = refCore[size_t(refVIndex)];

    int incidentAxis = 0;
    double bestAlignment = -1;
    for (int axis = 0; axis < 3; ++axis) {
        const double alignment = std::abs(dot(incAxes[size_t(axis)], refNormal));
        if (alignment > bestAlignment) {
            bestAlignment = alignment;
            incidentAxis = axis;
        }
    }
    const int incidentSign = dot(incAxes[size_t(incidentAxis)], refNormal) > 0
        ? -1
        : 1;
    std::vector<Vec3> polygon = faceVertices(incident, incidentAxis, incidentSign);
    polygon = clipPolygon(polygon, refU, dot(refU, refCenter) + refUExtent);
    polygon = clipPolygon(polygon, -refU, dot(-refU, refCenter) + refUExtent);
    polygon = clipPolygon(polygon, refV, dot(refV, refCenter) + refVExtent);
    polygon = clipPolygon(polygon, -refV, dot(-refV, refCenter) + refVExtent);

    const double radiusSum = reference.rounding + incident.rounding;
    for (const Vec3& incidentCorePoint : polygon) {
        const double coreSeparation = dot(incidentCorePoint - refCenter, refNormal);
        const double surfaceSeparation = coreSeparation - radiusSum;
        if (surfaceSeparation > speculativeMargin) continue;
        const Vec3 referenceCorePoint =
            incidentCorePoint - coreSeparation * refNormal;
        const Vec3 pointA = referenceCorePoint + reference.rounding * refNormal;
        const Vec3 pointB = incidentCorePoint - incident.rounding * refNormal;
        contacts.push_back({0.5 * (pointA + pointB), -surfaceSeparation});
    }
    return !contacts.empty();
}

bool buildRoundedBoxManifold(const Body& a,
                             const Body& b,
                             double dt,
                             Simulator::ContactManifold& manifold) {
    const SatResult sat = queryRoundedBoxes(a, b);
    const Vec3 centerRelativeVelocity = b.v - a.v;
    const double closingSpeed = std::max(0.0, -dot(centerRelativeVelocity, sat.normal));
    const double speculativeMargin = 0.00018 + closingSpeed * dt;
    if (!sat.overlap && sat.separation > speculativeMargin) return false;

    manifold.normal = sat.normal;
    std::vector<std::pair<Vec3, double>> contacts;
    contacts.reserve(8);

    if (sat.kind == SatKind::FaceA) {
        buildFaceContacts(a,
                          b,
                          sat.axisA,
                          sat.normal,
                          speculativeMargin,
                          contacts);
    } else if (sat.kind == SatKind::FaceB) {
        buildFaceContacts(b,
                          a,
                          sat.axisB,
                          -sat.normal,
                          speculativeMargin,
                          contacts);
    } else {
        const auto segmentA = edgeSegment(a, sat.axisA, sat.normal);
        const auto segmentB = edgeSegment(b, sat.axisB, -sat.normal);
        const auto closest = closestSegmentPoints(
            segmentA.first,
            segmentA.second,
            segmentB.first,
            segmentB.second);
        const Vec3 pointA = closest.first + a.rounding * sat.normal;
        const Vec3 pointB = closest.second - b.rounding * sat.normal;
        const double separation = dot(pointB - pointA, sat.normal);
        if (separation <= speculativeMargin) {
            contacts.push_back({0.5 * (pointA + pointB), -separation});
        }
    }

    if (contacts.empty()) {
        const Vec3 pointA = supportRoundedBox(a, sat.normal);
        const Vec3 pointB = supportRoundedBox(b, -sat.normal);
        const double separation = dot(pointB - pointA, sat.normal);
        if (separation > speculativeMargin) return false;
        contacts.push_back({0.5 * (pointA + pointB), -separation});
    }

    // Keep up to four well-spaced points. This prevents dense face contacts from
    // degenerating into one rocking point while keeping the solver bounded.
    while (!contacts.empty() && manifold.pointCount < 4) {
        size_t selected = 0;
        if (manifold.pointCount > 0) {
            double bestDistance = -1;
            for (size_t candidate = 0; candidate < contacts.size(); ++candidate) {
                double nearest = INF;
                for (int existing = 0; existing < manifold.pointCount; ++existing) {
                    nearest = std::min(
                        nearest,
                        lengthSquared(contacts[candidate].first -
                                      manifold.points[size_t(existing)].point));
                }
                if (nearest > bestDistance) {
                    bestDistance = nearest;
                    selected = candidate;
                }
            }
        }
        Simulator::ContactPoint point;
        point.point = contacts[selected].first;
        point.penetration = contacts[selected].second;
        point.feature = uint32_t(manifold.pointCount) |
            (uint32_t(sat.kind) << 8) |
            (uint32_t(std::max(0, sat.axisA)) << 12) |
            (uint32_t(std::max(0, sat.axisB)) << 16);
        manifold.points[size_t(manifold.pointCount++)] = point;
        contacts.erase(contacts.begin() + ptrdiff_t(selected));
    }
    return manifold.pointCount > 0;
}

bool buildFloorManifold(const Body& body, Simulator::ContactManifold& manifold) {
    const auto corners = coreCorners(body);
    double minimumSurfaceY = INF;
    for (const Vec3& corner : corners) {
        minimumSurfaceY = std::min(minimumSurfaceY, corner.y - body.rounding);
    }
    if (minimumSurfaceY >= 0.00035) return false;

    const double band = 0.00075;
    std::vector<std::pair<Vec3, double>> candidates;
    candidates.reserve(4);
    for (const Vec3& corner : corners) {
        const double surfaceY = corner.y - body.rounding;
        if (surfaceY <= minimumSurfaceY + band) {
            candidates.push_back({Vec3{corner.x, 0, corner.z}, -surfaceY});
        }
    }
    if (candidates.empty()) {
        const Vec3 support = supportRoundedBox(body, {0, -1, 0});
        candidates.push_back({Vec3{support.x, 0, support.z}, -support.y});
    }

    manifold.normal = {0, -1, 0};
    while (!candidates.empty() && manifold.pointCount < 4) {
        size_t selected = 0;
        if (manifold.pointCount > 0) {
            double bestDistance = -1;
            for (size_t candidate = 0; candidate < candidates.size(); ++candidate) {
                double nearest = INF;
                for (int existing = 0; existing < manifold.pointCount; ++existing) {
                    nearest = std::min(
                        nearest,
                        lengthSquared(candidates[candidate].first -
                                      manifold.points[size_t(existing)].point));
                }
                if (nearest > bestDistance) {
                    bestDistance = nearest;
                    selected = candidate;
                }
            }
        }
        Simulator::ContactPoint point;
        point.point = candidates[selected].first;
        point.penetration = candidates[selected].second;
        point.feature = uint32_t(manifold.pointCount) | 0xF0000000u;
        manifold.points[size_t(manifold.pointCount++)] = point;
        candidates.erase(candidates.begin() + ptrdiff_t(selected));
    }
    return manifold.pointCount > 0;
}

MagneticGeometry magneticGeometry(const Body& body,
                                   const SimulationConfig& cfg,
                                   double magnetization) {
    MagneticGeometry geometry;

    // Opposite faces deliberately share the same tangent basis. Therefore two
    // neighboring A/B modules in the authored checkerboard present exactly
    // complementary 2x2 magnetic pixels when their broad faces line up.
    const std::array<Vec3, 6> localNormal{{
        { 1, 0, 0}, {-1, 0, 0},
        { 0, 1, 0}, { 0,-1, 0},
        { 0, 0, 1}, { 0, 0,-1}
    }};
    const std::array<Vec3, 6> localU{{
        {0,1,0}, {0,1,0},
        {1,0,0}, {1,0,0},
        {1,0,0}, {1,0,0}
    }};
    const std::array<Vec3, 6> localV{{
        {0,0,1}, {0,0,1},
        {0,0,1}, {0,0,1},
        {0,1,0}, {0,1,0}
    }};

    const double classSign = body.moduleClass == 0 ? 1.0 : -1.0;
    const double chargeMagnitude = magnetization * body.strengthVariation;
    for (int faceIndex = 0; faceIndex < 6; ++faceIndex) {
        MagneticFace& face = geometry.faces[size_t(faceIndex)];
        const Vec3 nLocal = localNormal[size_t(faceIndex)];
        const Vec3 uLocal = localU[size_t(faceIndex)];
        const Vec3 vLocal = localV[size_t(faceIndex)];
        face.normal = normalize(body.q.rotate(nLocal));
        face.tangentU = normalize(body.q.rotate(uLocal));
        face.tangentV = normalize(body.q.rotate(vLocal));
        const Vec3 faceOffset{
            nLocal.x * body.half.x,
            nLocal.y * body.half.y,
            nLocal.z * body.half.z
        };
        face.center = body.p + body.q.rotate(faceOffset);

        int patch = 0;
        for (int su : {-1, 1}) {
            for (int sv : {-1, 1}) {
                face.patchPosition[size_t(patch)] = face.center +
                    double(su) * cfg.facePixelOffset * face.tangentU +
                    double(sv) * cfg.facePixelOffset * face.tangentV;
                // Balanced checkerboard: + - / - +. It has zero net pole and
                // no global dipole, but a strong localized docking field.
                face.patchCharge[size_t(patch)] =
                    classSign * double(su * sv) * chargeMagnitude;
                ++patch;
            }
        }
    }
    return geometry;
}

Interaction chargeInteraction(const Body& a,
                              const Body& b,
                              const Vec3* positionA,
                              const double* chargeA,
                              int countA,
                              const Vec3* positionB,
                              const double* chargeB,
                              int countB,
                              const SimulationConfig& cfg) {
    Interaction result;
    const double softening2 = sq(cfg.poleSoftening);
    for (int ia = 0; ia < countA; ++ia) {
        for (int ib = 0; ib < countB; ++ib) {
            const Vec3 delta = positionB[ib] - positionA[ia];
            const double rawDistance2 = lengthSquared(delta);
            const double distance2 = rawDistance2 + softening2;
            const double inverseDistance = 1.0 / std::sqrt(distance2);
            const double inverseDistance3 = inverseDistance / distance2;
            const double chargeProduct = chargeA[ia] * chargeB[ib];
            const Vec3 forceOnB =
                cfg.magneticK * chargeProduct * inverseDistance3 * delta;
            result.potential += cfg.magneticK * chargeProduct * inverseDistance;
            result.forceOnB += forceOnB;
            result.torqueA += cross(positionA[ia] - a.p, -forceOnB);
            result.torqueB += cross(positionB[ib] - b.p, forceOnB);
            if (chargeProduct < 0) {
                const double distance = std::sqrt(rawDistance2);
                if (distance < result.nearestOpposite) {
                    result.nearestOpposite = distance;
                    result.nearestPointA = positionA[ia];
                    result.nearestPointB = positionB[ib];
                }
            }
        }
    }
    return result;
}

std::array<int, 2> bestFacesToward(const MagneticGeometry& geometry,
                                   const Vec3& direction) {
    std::array<int, 2> best{{0, 1}};
    double first = -INF;
    double second = -INF;
    for (int face = 0; face < 6; ++face) {
        const double score = dot(geometry.faces[size_t(face)].normal, direction);
        if (score > first) {
            second = first;
            best[1] = best[0];
            first = score;
            best[0] = face;
        } else if (score > second) {
            second = score;
            best[1] = face;
        }
    }
    return best;
}

int stableFaceToward(const MagneticGeometry& geometry,
                     const Vec3& direction,
                     int previousFace) {
    const int bestFace = bestFacesToward(geometry, direction)[0];
    if (previousFace < 0 || previousFace >= 6) return bestFace;
    const double bestScore = dot(
        geometry.faces[size_t(bestFace)].normal,
        direction);
    const double previousScore = dot(
        geometry.faces[size_t(previousFace)].normal,
        direction);
    // A 0.12 cosine hysteresis band prevents axis selection from toggling as a
    // resting cube rocks by microradians. A face changes only during a clearly
    // resolved rotation, where the switch is physically readable rather than
    // appearing as high-frequency magnetic chatter.
    if (previousScore > 0.05 && previousScore >= bestScore - 0.12)
        return previousFace;
    return bestFace;
}

Interaction evaluateMagneticPair(
                                 const Body& a,
                                 const Body& b,
                                 const MagneticGeometry& ga,
                                 const MagneticGeometry& gb,
                                 const SimulationConfig& cfg,
                                 std::unordered_map<uint64_t, uint16_t, Int3Hash>& faceHistory,
                                 const std::unordered_map<uint64_t, uint16_t, Int3Hash>& capturedFacePairs) {
    const Vec3 centerDelta = b.p - a.p;
    const double centerDistance = length(centerDelta);
    if (centerDistance < 1e-9 || centerDistance >= cfg.farCut) return {};
    const Vec3 centerDirection = centerDelta / centerDistance;

    const uint64_t key = pairKey(a.id, b.id);
    int previousA = -1;
    int previousB = -1;
    const auto old = faceHistory.find(key);
    if (old != faceHistory.end()) {
        previousA = int(old->second & 0x7u);
        previousB = int((old->second >> 3) & 0x7u);
    }

    // A physically seated complementary face pair keeps the same broad faces
    // until a real separation occurs. Without this capture hysteresis, an
    // almost static cube can alternate between equally good local axes and the
    // magnetic torque becomes a high-frequency discontinuity.
    bool captured = false;
    int faceIndexA = -1;
    int faceIndexB = -1;
    const auto capturedIt = capturedFacePairs.find(key);
    if (capturedIt != capturedFacePairs.end()) {
        faceIndexA = int(capturedIt->second & 0x7u);
        faceIndexB = int((capturedIt->second >> 3) & 0x7u);
        captured = faceIndexA >= 0 && faceIndexA < 6 &&
            faceIndexB >= 0 && faceIndexB < 6;
    }
    if (!captured) {
        faceIndexA = stableFaceToward(ga, centerDirection, previousA);
        faceIndexB = stableFaceToward(gb, -centerDirection, previousB);
    }
    faceHistory[key] = uint16_t(faceIndexA | (faceIndexB << 3));

    const MagneticFace& fa = ga.faces[size_t(faceIndexA)];
    const MagneticFace& fb = gb.faces[size_t(faceIndexB)];
    const double towardA = dot(fa.normal, centerDirection);
    const double towardB = dot(fb.normal, -centerDirection);
    const double opposingNormals = dot(fa.normal, -fb.normal);
    const double faceDistance = length(fb.center - fa.center);
    if (towardA <= 0.02 || towardB <= 0.02 || opposingNormals <= -0.20 ||
        faceDistance >= cfg.faceInteractionStart) return {};

    const double directional = smootherstep(0.02, 0.82, towardA) *
        smootherstep(0.02, 0.82, towardB);
    const double normalAgreement = smootherstep(-0.20, 0.78,
                                                opposingNormals);
    const double distanceWeight = 1.0 - smootherstep(
        0.030, cfg.faceInteractionStart, faceDistance);
    const double weight = directional * normalAgreement * distanceWeight;
    if (weight <= 1e-6) return {};

    Interaction combined = chargeInteraction(
        a, b,
        fa.patchPosition.data(), fa.patchCharge.data(), 4,
        fb.patchPosition.data(), fb.patchCharge.data(), 4,
        cfg);
    combined.potential *= weight;
    combined.forceOnB *= weight;
    combined.torqueA *= weight;
    combined.torqueB *= weight;
    combined.captureNormal = normalize(fb.center - fa.center);
    combined.captureQuality = weight * saturate(-combined.potential / 0.0024);
    combined.faceIndexA = faceIndexA;
    combined.faceIndexB = faceIndexB;
    combined.facePairCaptured = captured;
    // Broad face-center damping points are stable even when the closest one of
    // the four magnetic pixels changes within a seated connection.
    combined.nearestPointA = fa.center;
    combined.nearestPointB = fb.center;

    if (centerDistance > cfg.farFadeStart) {
        const SmoothValue fade = smootherstepWithDerivative(
            cfg.farFadeStart, cfg.farCut, centerDistance);
        const double keep = 1.0 - fade.value;
        const double keepDerivative = -fade.derivative;
        combined.forceOnB = keep * combined.forceOnB -
            keepDerivative * combined.potential * centerDirection;
        combined.torqueA *= keep;
        combined.torqueB *= keep;
        combined.potential *= keep;
        combined.captureQuality *= keep;
    }
    return combined;
}

} // namespace

const char* phaseName(AssemblyPhase phase) {
    switch (phase) {
        case AssemblyPhase::FixtureHold: return "equilibrated_hold";
        case AssemblyPhase::FixtureRetract: return "vertical_trapdoor_release";
        case AssemblyPhase::PermanentCollapse: return "dice_collapse";
        case AssemblyPhase::Settling: return "settling";
        case AssemblyPhase::Reserved4: return "unused";
        case AssemblyPhase::Reserved5: return "unused";
        case AssemblyPhase::Complete: return "settled";
    }
    return "unknown";
}

Simulator::Simulator(SimulationConfig config) : cfg_(config) {
    initializeScene();
}

void Simulator::loadState(const std::vector<Snapshot>& snapshots,
                          AssemblyPhase phase) {
    if (snapshots.size() < bodies_.size()) {
        throw std::runtime_error("resume snapshot does not contain every body");
    }
    for (size_t i = 0; i < bodies_.size(); ++i) {
        Body& body = bodies_[i];
        const Snapshot& snapshot = snapshots[i];
        body.p = snapshot.p;
        body.q = normalized(snapshot.q);
        body.v = snapshot.v;
        body.w = snapshot.w;
        body.half = snapshot.half;
        body.material = snapshot.material;
        body.moduleClass = snapshot.moduleClass;
        body.layer = snapshot.layer;
        body.slot = snapshot.slot;
        // Cache snapshots do not store the sleep flag. Exact-zero motion is
        // the serialized signature of a locally sleeping rigid body; preserve
        // it when a long simulation is continued in a second process.
        body.sleeping =
            lengthSquared(body.v) < 1e-24 && lengthSquared(body.w) < 1e-24;
        body.sleepTimer = body.sleeping ? cfg_.sleepDelay : 0.0;
        body.contactCount = 0;
        body.supportCount = 0;
    }
    phase_ = phase;
    phaseStart_ = 0;
    time_ = 0;
    preRolling_ = false;
    stateLoaded_ = true;
    manifoldCache_.clear();
    magneticFaceHistory_.clear();
    capturedFacePairs_.clear();
    accumulatedMagneticWork_ = 0;
    lastMaxPenetration_ = 0;
    lastCloseMagneticPairs_ = 0;
    lastSnappedPolePairs_ = 0;
    lastContactManifolds_ = 0;

    double meanHeight = 0;
    for (int i = 0; i < kModuleCount; ++i) {
        meanHeight += bodies_[size_t(i)].p.y;
    }
    initialMeanHeight_ = std::max(1e-6, meanHeight / double(kModuleCount));
}

void Simulator::initializeScene() {
    initializeModules();
    initializePedestal();
}

void Simulator::initializeModules() {
    bodies_.clear();
    bodies_.reserve(kModuleCount + 4);

    const double fullSize = 0.0160;
    const double halfSize = 0.5 * fullSize;
    const double gap = 0.00028;
    const double pitch = fullSize + gap;
    const double platformTop = 0.065;

    auto quarterTurn = [](int axis, int turns) {
        const double angle = 0.5 * PI * double(turns & 3);
        if (axis == 0) return quatAxisAngle({1, 0, 0}, angle);
        if (axis == 1) return quatAxisAngle({0, 1, 0}, angle);
        return quatAxisAngle({0, 0, 1}, angle);
    };

    for (int layer = 0; layer < kLayerCount; ++layer) {
        for (int z = 0; z < kTowerZ; ++z) {
            for (int x = 0; x < kTowerX; ++x) {
                Body body;
                body.id = int(bodies_.size());
                body.kind = BodyKind::Module;
                body.layer = layer;
                body.slot = z * kTowerX + x;
                body.half = {halfSize, halfSize, halfSize};
                body.rounding = 0.00155;

                const uint64_t h0 = splitmix64(
                    cfg_.seed ^ uint64_t(body.id + 1) * 0x9E3779B97F4A7C15ULL);
                const int rx = int((h0 >> 4) & 3u);
                const int ry = int((h0 >> 11) & 3u);
                const int rz = int((h0 >> 19) & 3u);
                body.q = normalized(
                    quarterTurn(2, rz) * quarterTurn(1, ry) * quarterTurn(0, rx));
                body.moduleClass = int((h0 >> 27) % 24u);

                const double leanX = double(layer) * 1.15e-5;
                const double leanZ = -double(layer) * 0.60e-5;
                const double px =
                    (double(x) - 0.5 * double(kTowerX - 1)) * pitch + leanX;
                const double pz =
                    (double(z) - 0.5 * double(kTowerZ - 1)) * pitch + leanZ;
                const double py = platformTop + halfSize + double(layer) * pitch;
                body.p = {px, py, pz};

                const double volume = fullSize * fullSize * fullSize;
                const double tolerance =
                    double(uint32_t(h0 >> 32)) / 4294967295.0;
                const double density = cfg_.moduleDensity *
                    (0.992 + 0.016 * tolerance);
                body.mass = density * volume;
                body.invMass = 1.0 / body.mass;
                const double inertia = body.mass *
                    (fullSize * fullSize + fullSize * fullSize) / 12.0;
                body.invILocal = Vec3(1.0 / inertia);
                body.strengthVariation = 0.0;

                const int palette = int((h0 >> 45) & 31u);
                body.material = palette < 21 ? 0 : (palette < 28 ? 1 : 2);
                body.initialP = body.p;
                body.initialQ = body.q;
                bodies_.push_back(body);
            }
        }
    }

    if (int(bodies_.size()) != kModuleCount) {
        throw std::runtime_error("dice generator produced wrong body count");
    }
}

void Simulator::initializePedestal() {
    coils_[0].current = 0;

    Body pedestal;
    pedestal.id = int(bodies_.size());
    pedestal.kind = BodyKind::Pedestal;
    pedestal.anchored = true;
    pedestal.collidable = false;
    pedestal.invMass = 0;
    pedestal.mass = 0;
    pedestal.p = {0, -0.055, 0};
    pedestal.half = {0.235, 0.055, 0.165};
    pedestal.rounding = 0.020;
    pedestal.material = 5;
    bodies_.push_back(pedestal);

    Body trapdoor;
    trapdoor.id = int(bodies_.size());
    trapdoor.kind = BodyKind::Pedestal;
    trapdoor.anchored = true;
    trapdoor.collidable = true;
    trapdoor.invMass = 0;
    trapdoor.mass = 0;
    trapdoor.p = {0, 0.052, 0};
    trapdoor.initialP = trapdoor.p;
    trapdoor.half = {0.098, 0.013, 0.058};
    trapdoor.rounding = 0.006;
    trapdoor.material = 5;
    trapdoorBodyIndex_ = trapdoor.id;
    bodies_.push_back(trapdoor);
}

void Simulator::enterPhase(AssemblyPhase phase) {
    phase_ = phase;
    phaseStart_ = time_;

    // The support begins moving at this physical event. Any dice that were
    // resting during the unrecorded equilibrium preroll become dynamic here;
    // there is no later end-of-shot global sleep or freeze.
    if (phase == AssemblyPhase::FixtureRetract) {
        for (int i = 0; i < kModuleCount; ++i) wake(bodies_[size_t(i)]);
    }
}

void Simulator::updatePhase(double) {
    if (preRolling_) return;
    const double elapsed = time_ - phaseStart_;
    switch (phase_) {
        case AssemblyPhase::FixtureHold:
            if (elapsed >= cfg_.initialHold) {
                enterPhase(AssemblyPhase::FixtureRetract);
            }
            break;
        case AssemblyPhase::FixtureRetract:
            if (elapsed >= cfg_.releaseDuration) {
                enterPhase(AssemblyPhase::PermanentCollapse);
            }
            break;
        case AssemblyPhase::PermanentCollapse:
            if (elapsed >= cfg_.collapseLatest) {
                enterPhase(AssemblyPhase::Settling);
            }
            break;
        case AssemblyPhase::Settling:
            if (elapsed >= cfg_.settlingDuration) {
                enterPhase(AssemblyPhase::Complete);
            }
            break;
        case AssemblyPhase::Reserved4:
        case AssemblyPhase::Reserved5:
            enterPhase(AssemblyPhase::Settling);
            break;
        case AssemblyPhase::Complete:
            break;
    }
}

double Simulator::globalMagnetization() const {
    return cfg_.permanentMagnetization;
}

double Simulator::bodyMagnetization(const Body&) const {
    return cfg_.permanentMagnetization;
}

int Simulator::chooseSubsteps(double frameDt) const {
    double maximumSpeed = 0;
    double maximumSpin = 0;
    for (int i = 0; i < kModuleCount; ++i) {
        maximumSpeed = std::max(maximumSpeed, length(bodies_[size_t(i)].v));
        maximumSpin = std::max(maximumSpin, length(bodies_[size_t(i)].w));
    }
    const double minimumThickness = 0.0160;
    const double travelDemand = maximumSpeed * frameDt /
        (0.22 * minimumThickness);
    const double rotationDemand = maximumSpin * frameDt / 0.12;
    const int additional = int(std::ceil(
        3.2 * std::max(travelDemand, rotationDemand)));
    return std::clamp(cfg_.baseSubsteps + additional,
                      cfg_.baseSubsteps,
                      cfg_.maxSubsteps);
}

void Simulator::runPreRoll() {
    // Establish a real contact equilibrium under the exact equations used in
    // the recorded shot. The support remains fixed and all dice are dynamic;
    // no authored-pose reset or scene-wide wake event occurs at release.
    for (int i = 0; i < kModuleCount; ++i) {
        Body& body = bodies_[size_t(i)];
        body.p = body.initialP;
        body.q = body.initialQ;
        body.v = Vec3(0);
        body.w = Vec3(0);
        body.sleeping = false;
        body.sleepTimer = 0;
        body.sleepReferenceMagneticForce = Vec3(0);
        body.sleepReferenceMagneticTorque = Vec3(0);
    }

    Body& trapdoor = bodies_[size_t(trapdoorBodyIndex_)];
    trapdoor.p = trapdoor.initialP;
    trapdoor.v = Vec3(0);
    trapdoor.w = Vec3(0);

    preRolling_ = true;
    time_ = 0;
    phase_ = AssemblyPhase::FixtureHold;
    phaseStart_ = 0;
    accumulatedMagneticWork_ = 0;

    const double preRollDt = 1.0 / 360.0;
    const int preRollSteps = std::max(
        0,
        int(std::ceil(cfg_.preRollSeconds / preRollDt)));
    for (int stepIndex = 0; stepIndex < preRollSteps; ++stepIndex) {
        step(preRollDt);
    }

    // Preserve the physically settled transforms and sleep states, but restart
    // cinematic time at zero with the support in its exact initial pose.
    trapdoor.p = trapdoor.initialP;
    trapdoor.v = Vec3(0);
    trapdoor.w = Vec3(0);
    preRolling_ = false;
    time_ = 0;
    phase_ = AssemblyPhase::FixtureHold;
    phaseStart_ = 0;
    manifoldCache_.clear();

    double meanHeight = 0;
    for (int i = 0; i < kModuleCount; ++i) {
        meanHeight += bodies_[size_t(i)].p.y;
    }
    initialMeanHeight_ = meanHeight / double(kModuleCount);
}

void Simulator::step(double dt) {
    updatePhase(dt);

    // Keep the realized trajectory for each die. The position projection below
    // may cancel a low-energy attempted motion; reconciling velocity with the
    // realized displacement prevents a supported pile from repeatedly falling
    // into itself and being projected back out (the visible "breathing" mode).
    std::vector<Vec3> stepStartPosition{size_t(kModuleCount)};
    for (int i = 0; i < kModuleCount; ++i) {
        stepStartPosition[size_t(i)] = bodies_[size_t(i)].p;
    }

    Body& trapdoor = bodies_[size_t(trapdoorBodyIndex_)];
    double retract = 0;
    if (!preRolling_) {
        if (phase_ == AssemblyPhase::FixtureRetract) {
            retract = smootherstep(0.0,
                                   cfg_.releaseDuration,
                                   time_ - phaseStart_);
        } else if (phase_ == AssemblyPhase::PermanentCollapse ||
                   phase_ == AssemblyPhase::Settling ||
                   phase_ == AssemblyPhase::Complete) {
            retract = 1.0;
        }
    }
    const Vec3 newTrapdoorPosition = trapdoor.initialP +
        Vec3{0, -cfg_.trapdoorTravel * retract, 0};
    trapdoor.v = (newTrapdoorPosition - trapdoor.p) / std::max(1e-9, dt);
    trapdoor.w = Vec3(0);
    trapdoor.p = newTrapdoorPosition;

    for (Body& body : bodies_) {
        body.force = Vec3(0);
        body.torque = Vec3(0);
        body.contactCount = 0;
        body.supportCount = 0;
        if (body.kind == BodyKind::Module && body.invMass > 0) {
            body.force.y -= body.mass * cfg_.gravity;
        }
    }

    const bool magnetismEnabled =
        cfg_.permanentMagnetization > 0.0 &&
        cfg_.magneticK > 0.0 &&
        cfg_.farCut > 0.0;
    std::vector<MagneticGeometry> magneticGeometryCache;
    if (magnetismEnabled) {
        magneticGeometryCache.resize(size_t(kModuleCount));
        for (int i = 0; i < kModuleCount; ++i) {
            magneticGeometryCache[size_t(i)] = magneticGeometry(
                bodies_[size_t(i)],
                cfg_,
                bodyMagnetization(bodies_[size_t(i)]));
        }
    }

    std::unordered_map<int64_t, std::vector<int>, Int3Hash> grid;
    grid.reserve(kModuleCount * 2);
    for (int i = 0; i < kModuleCount; ++i) {
        const Vec3& p = bodies_[size_t(i)].p;
        grid[cellKey(int(std::floor(p.x / cfg_.broadCell)),
                     int(std::floor(p.y / cfg_.broadCell)),
                     int(std::floor(p.z / cfg_.broadCell)))].push_back(i);
    }

    std::vector<PairCandidate> magneticPairs;
    std::vector<PairCandidate> contactPairs;
    magneticPairs.reserve(kModuleCount * 42);
    contactPairs.reserve(kModuleCount * 16);
    const int reach = magnetismEnabled
        ? std::max(1, int(std::ceil(cfg_.farCut / cfg_.broadCell)))
        : 1;
    const double magneticCutoff2 = magnetismEnabled ? sq(cfg_.farCut) : 0.0;

    for (int i = 0; i < kModuleCount; ++i) {
        const Body& a = bodies_[size_t(i)];
        const int cx = int(std::floor(a.p.x / cfg_.broadCell));
        const int cy = int(std::floor(a.p.y / cfg_.broadCell));
        const int cz = int(std::floor(a.p.z / cfg_.broadCell));
        for (int dz = -reach; dz <= reach; ++dz) {
            for (int dy = -reach; dy <= reach; ++dy) {
                for (int dx = -reach; dx <= reach; ++dx) {
                    const auto it = grid.find(cellKey(cx + dx, cy + dy, cz + dz));
                    if (it == grid.end()) continue;
                    for (int j : it->second) {
                        if (j <= i) continue;
                        const Body& b = bodies_[size_t(j)];
                        const double distance2 = lengthSquared(b.p - a.p);
                        if (magnetismEnabled && distance2 < magneticCutoff2) {
                            magneticPairs.push_back({i, j, distance2});
                        }
                        const double contactRadius =
                            length(a.half) + length(b.half) +
                            cfg_.contactBroadMargin;
                        if (distance2 < contactRadius * contactRadius) {
                            contactPairs.push_back({i, j, distance2});
                        }
                    }
                }
            }
        }
    }

    // The vertically moving trapdoor is the only kinematic contact body.
    for (int i = 0; i < kModuleCount; ++i) {
        const Body& module = bodies_[size_t(i)];
        const double radius = length(module.half) + length(trapdoor.half) +
            cfg_.contactBroadMargin;
        const double distance2 = lengthSquared(trapdoor.p - module.p);
        if (distance2 < radius * radius) {
            contactPairs.push_back({i, trapdoorBodyIndex_, distance2});
        }
    }

    lastCloseMagneticPairs_ = 0;
    lastSnappedPolePairs_ = 0;
    std::vector<Vec3> magneticForce(size_t(kModuleCount), Vec3(0));
    std::vector<Vec3> magneticTorque(size_t(kModuleCount), Vec3(0));

    std::unordered_set<uint64_t, Int3Hash> activeMagneticKeys;
    activeMagneticKeys.reserve(magneticPairs.size());
    for (const PairCandidate& pair : magneticPairs) {
        Body& a = bodies_[size_t(pair.a)];
        Body& b = bodies_[size_t(pair.b)];
        const MagneticGeometry& ga = magneticGeometryCache[size_t(pair.a)];
        const MagneticGeometry& gb = magneticGeometryCache[size_t(pair.b)];
        const uint64_t magneticKey = pairKey(pair.a, pair.b);
        activeMagneticKeys.insert(magneticKey);
        Interaction interaction = evaluateMagneticPair(
            a, b, ga, gb, cfg_, magneticFaceHistory_, capturedFacePairs_);
        a.force -= interaction.forceOnB;
        b.force += interaction.forceOnB;
        a.torque += interaction.torqueA;
        b.torque += interaction.torqueB;
        magneticForce[size_t(pair.a)] -= interaction.forceOnB;
        magneticForce[size_t(pair.b)] += interaction.forceOnB;
        magneticTorque[size_t(pair.a)] += interaction.torqueA;
        magneticTorque[size_t(pair.b)] += interaction.torqueB;
        accumulatedMagneticWork_ += std::max(
            0.0,
            dot(interaction.forceOnB, b.v - a.v)) * dt;

        const double centerDistance = std::sqrt(pair.distance2);
        if (centerDistance < cfg_.farCut && interaction.captureQuality > 0.002)
            ++lastCloseMagneticPairs_;
        if (interaction.nearestOpposite < cfg_.captureGap &&
            interaction.captureQuality > 0.035)
            ++lastSnappedPolePairs_;

        // A broad complementary face becomes a persistent captured pair only
        // after real surface seating. The pair is released with a much wider
        // gap threshold, providing physical hysteresis rather than repeatedly
        // attaching and detaching at one floating-point boundary.
        const SatResult contactState = queryRoundedBoxes(a, b);
        const double surfaceGap = contactState.overlap
            ? 0.0
            : std::max(0.0, contactState.separation);
        bool capturedNow = capturedFacePairs_.find(magneticKey) !=
            capturedFacePairs_.end();
        if (capturedNow) {
            if (surfaceGap > 0.00115 ||
                interaction.captureQuality < 0.0015 ||
                interaction.faceIndexA < 0 || interaction.faceIndexB < 0) {
                capturedFacePairs_.erase(magneticKey);
                capturedNow = false;
            }
        } else if (surfaceGap < 0.00028 &&
                   interaction.captureQuality > 0.030 &&
                   interaction.faceIndexA >= 0 && interaction.faceIndexB >= 0) {
            capturedFacePairs_[magneticKey] = uint16_t(
                interaction.faceIndexA | (interaction.faceIndexB << 3));
            capturedNow = true;
        }

        // Local capture loss is continuous in gap and alignment and is applied
        // only around a physically seated pair. Captured pairs receive stronger
        // eddy-current/contact loss, suppressing endless microscopic rocking
        // without damping free flight or the initial magnetic snap.
        if (interaction.nearestOpposite < 1.35 * cfg_.captureGap &&
            interaction.captureQuality > 1e-5) {
            const double seatWeight = 1.0 - smootherstep(
                0.00016,
                capturedNow ? 0.00105 : 0.00062,
                surfaceGap);
            const double qualityWeight = smootherstep(
                capturedNow ? 0.002 : 0.006,
                capturedNow ? 0.035 : 0.060,
                interaction.captureQuality);
            const double poleWeight = 1.0 - smootherstep(
                0.00018,
                capturedNow ? 1.75 * cfg_.captureGap : 1.35 * cfg_.captureGap,
                interaction.nearestOpposite);
            const double captureMultiplier = capturedNow ? 1.85 : 1.0;
            const double dampingWeight = captureMultiplier *
                seatWeight * qualityWeight * poleWeight;
            if (dampingWeight > 1e-8) {
                const Vec3 pointA = interaction.nearestPointA;
                const Vec3 pointB = interaction.nearestPointB;
                Vec3 n = interaction.captureNormal;
                if (lengthSquared(n) < 1e-12) n = normalize(pointB - pointA);
                const Vec3 relativeVelocity =
                    pointVelocity(b, pointB) - pointVelocity(a, pointA);
                const double normalSpeed = dot(relativeVelocity, n);
                const Vec3 tangentialVelocity =
                    relativeVelocity - normalSpeed * n;
                const Vec3 dampingOnB = -dampingWeight * (
                    cfg_.magneticNormalDamping * normalSpeed * n +
                    cfg_.magneticTangentialDamping * tangentialVelocity);
                applyForceAt(a, -dampingOnB, pointA);
                applyForceAt(b, dampingOnB, pointB);
                const Vec3 relativeSpin = b.w - a.w;
                const Vec3 dampingTorque = -cfg_.magneticAngularDamping *
                    dampingWeight * relativeSpin;
                a.torque -= dampingTorque;
                b.torque += dampingTorque;
            }
        }
    }

    // A pair that left the magnetic broad phase cannot remain captured.
    for (auto it = capturedFacePairs_.begin(); it != capturedFacePairs_.end();) {
        if (activeMagneticKeys.find(it->first) == activeMagneticKeys.end())
            it = capturedFacePairs_.erase(it);
        else
            ++it;
    }

    // Wake sleeping bodies only when their magnetic load changes materially.
    // Constant balanced magnetic load is allowed to remain asleep.
    if (!preRolling_ && phase_ != AssemblyPhase::FixtureHold) {
        for (int i = 0; i < kModuleCount; ++i) {
            Body& body = bodies_[size_t(i)];
            if (!body.sleeping) continue;
            const double forceChange = length(
                magneticForce[size_t(i)] - body.sleepReferenceMagneticForce);
            const double torqueChange = length(
                magneticTorque[size_t(i)] - body.sleepReferenceMagneticTorque);
            if (forceChange > cfg_.wakeMagneticForceDelta ||
                torqueChange > cfg_.wakeMagneticTorqueDelta) {
                wake(body);
            }
        }
    }

    const double linearDecay = std::exp(-cfg_.airDrag * dt);
    const double angularDecay = std::exp(-cfg_.angularAirDrag * dt);
    for (int i = 0; i < kModuleCount; ++i) {
        Body& body = bodies_[size_t(i)];
        if (body.sleeping) continue;
        body.v = (body.v + dt * body.invMass * body.force) * linearDecay;
        body.w = (body.w + dt * worldInvI(body, body.torque)) * angularDecay;
    }

    std::unordered_map<uint64_t, ContactManifold, Int3Hash> nextCache;
    nextCache.reserve(contactPairs.size() + kModuleCount);
    std::vector<uint64_t> activeKeys;
    activeKeys.reserve(contactPairs.size() + kModuleCount);
    lastMaxPenetration_ = 0;
    std::vector<double> bodyMaxPenetration(size_t(kModuleCount), 0.0);

    auto insertManifold = [&](ContactManifold manifold, uint64_t key) {
        const auto previous = manifoldCache_.find(key);
        if (previous != manifoldCache_.end()) {
            const ContactManifold& old = previous->second;
            manifold.lifetime = old.lifetime + 1;
            const double normalAgreement = dot(old.normal, manifold.normal);
            if (normalAgreement > 0.985) {
                // Persistent broad contacts keep a strongly hysteretic normal.
                // Warm starting is intentionally conservative: stale corner
                // impulses are a primary source of isolated late angular kicks.
                manifold.normal = normalize(
                    0.90 * old.normal + 0.10 * manifold.normal);
                constexpr double normalWarmScale = 0.82;
                constexpr double tangentWarmScale = 0.35;
                constexpr double matchRadius2 = 0.00030 * 0.00030;
                for (int pointIndex = 0;
                     pointIndex < manifold.pointCount;
                     ++pointIndex) {
                    double bestDistance = INF;
                    int bestPrevious = -1;
                    for (int oldIndex = 0;
                         oldIndex < old.pointCount;
                         ++oldIndex) {
                        const double distance = lengthSquared(
                            manifold.points[size_t(pointIndex)].point -
                            old.points[size_t(oldIndex)].point);
                        if (distance < bestDistance) {
                            bestDistance = distance;
                            bestPrevious = oldIndex;
                        }
                    }
                    if (bestPrevious >= 0 && bestDistance < matchRadius2) {
                        ContactPoint& point =
                            manifold.points[size_t(pointIndex)];
                        const ContactPoint& oldPoint =
                            old.points[size_t(bestPrevious)];
                        point.normalImpulse = normalWarmScale *
                            oldPoint.normalImpulse;
                        point.tangentImpulse = tangentWarmScale *
                            oldPoint.tangentImpulse;
                        point.tangentImpulse -= dot(
                            point.tangentImpulse,
                            manifold.normal) * manifold.normal;
                    }
                }
            }
        }
        for (int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex) {
            const double penetration = std::max(
                0.0,
                manifold.points[size_t(pointIndex)].penetration);
            lastMaxPenetration_ = std::max(lastMaxPenetration_, penetration);
            if (manifold.a >= 0 && manifold.a < kModuleCount) {
                bodyMaxPenetration[size_t(manifold.a)] = std::max(
                    bodyMaxPenetration[size_t(manifold.a)], penetration);
            }
            if (manifold.b >= 0 && manifold.b < kModuleCount) {
                bodyMaxPenetration[size_t(manifold.b)] = std::max(
                    bodyMaxPenetration[size_t(manifold.b)], penetration);
            }
        }
        if (manifold.a >= 0 && manifold.a < kModuleCount) {
            bodies_[size_t(manifold.a)].contactCount += manifold.pointCount;
        }
        if (manifold.b >= 0 && manifold.b < kModuleCount) {
            bodies_[size_t(manifold.b)].contactCount += manifold.pointCount;
        }
        // Track only load-bearing contacts.  Horizontal neighbours must not
        // keep an unsupported sleeping layer suspended after the trapdoor has
        // moved away.
        if (manifold.a >= 0 && manifold.a < kModuleCount &&
            -manifold.normal.y > 0.32) {
            bodies_[size_t(manifold.a)].supportCount += manifold.pointCount;
        }
        if (manifold.b >= 0 && manifold.b < kModuleCount &&
            manifold.normal.y > 0.32) {
            bodies_[size_t(manifold.b)].supportCount += manifold.pointCount;
        }
        nextCache.emplace(key, manifold);
        activeKeys.push_back(key);
    };

    for (const PairCandidate& pair : contactPairs) {
        Body& a = bodies_[size_t(pair.a)];
        Body& b = bodies_[size_t(pair.b)];
        ContactManifold manifold;
        manifold.a = pair.a;
        manifold.b = pair.b;
        if (buildRoundedBoxManifold(a, b, dt, manifold)) {
            insertManifold(manifold, pairKey(pair.a, pair.b));
        }
    }

    for (int i = 0; i < kModuleCount; ++i) {
        ContactManifold manifold;
        manifold.a = i;
        manifold.b = -1;
        if (buildFloorManifold(bodies_[size_t(i)], manifold)) {
            insertManifold(manifold, pairKey(i, 0x7ffffffe));
        }
    }
    lastContactManifolds_ = int(activeKeys.size());

    // Deterministic bottom-up constraint ordering acts as shock propagation:
    // support impulses are established at the floor before upper contacts are
    // solved. Random unordered-map iteration made the pile re-discover its
    // load path over many seconds and produced visible numerical creep.
    std::stable_sort(activeKeys.begin(), activeKeys.end(), [&](uint64_t lhs,
                                                               uint64_t rhs) {
        const ContactManifold& a = nextCache.at(lhs);
        const ContactManifold& b = nextCache.at(rhs);
        auto height = [&](const ContactManifold& m) {
            if (m.b < 0) return -1e6;
            double y = 0.0;
            for (int i = 0; i < m.pointCount; ++i)
                y += m.points[size_t(i)].point.y;
            return m.pointCount > 0 ? y / double(m.pointCount)
                                    : std::min(bodies_[size_t(m.a)].p.y,
                                               bodies_[size_t(m.b)].p.y);
        };
        const double ha = height(a);
        const double hb = height(b);
        if (std::abs(ha - hb) > 1e-9) return ha < hb;
        if (a.a != b.a) return a.a < b.a;
        return a.b < b.b;
    });

    // Sleeping is local and load-driven. The visible support release already
    // wakes every module simultaneously; propagating wakefulness through the
    // entire connected pile on every later substep prevented any region from
    // ever reaching an exact static state. A sleeping cube is instead woken by
    // a real contact impulse or a material change in magnetic force/torque.


    // Wake only at a real local impact. Resting support impulses are solved
    // against sleeping dice as static mass and therefore cannot create the
    // sleep/wake chatter that plagued the previous ending.
    for (uint64_t key : activeKeys) {
        ContactManifold& manifold = nextCache.at(key);
        Body& a = bodies_[size_t(manifold.a)];
        Body* b = manifold.b >= 0 ? &bodies_[size_t(manifold.b)] : nullptr;
        if (!b || (!a.sleeping && !b->sleeping) ||
            (a.sleeping && b->sleeping)) {
            continue;
        }

        bool meaningfulImpact = false;
        for (int pointIndex = 0;
             pointIndex < manifold.pointCount && !meaningfulImpact;
             ++pointIndex) {
            const ContactPoint& point = manifold.points[size_t(pointIndex)];
            const Vec3 velocityA = pointVelocity(a, point.point);
            const Vec3 velocityB = pointVelocity(*b, point.point);
            const Vec3 relativeVelocity = velocityB - velocityA;
            const double normalVelocity =
                dot(relativeVelocity, manifold.normal);
            const Vec3 tangentVelocity = relativeVelocity -
                normalVelocity * manifold.normal;
            const double penetration = std::max(0.0, point.penetration);
            meaningfulImpact =
                -normalVelocity > cfg_.wakeImpactNormalSpeed ||
                length(tangentVelocity) > cfg_.wakeImpactTangentSpeed ||
                penetration > cfg_.wakeImpactPenetration;
        }

        if (meaningfulImpact) {
            if (a.sleeping) wake(a);
            if (b->sleeping) wake(*b);
        }
    }

    auto inverseEffectiveMass = [&](const Body& a,
                                    const Body* b,
                                    const Vec3& point,
                                    const Vec3& direction) {
        double inverseMass = 0.0;
        if (!a.sleeping && a.invMass > 0.0) {
            const Vec3 rA = point - a.p;
            inverseMass += a.invMass +
                dot(direction,
                    cross(worldInvI(a, cross(rA, direction)), rA));
        }
        if (b && !b->sleeping && b->invMass > 0.0) {
            const Vec3 rB = point - b->p;
            inverseMass += b->invMass +
                dot(direction,
                    cross(worldInvI(*b, cross(rB, direction)), rB));
        }
        return std::max(1e-12, inverseMass);
    };

    // Warm start persistent contact manifolds.
    for (uint64_t key : activeKeys) {
        ContactManifold& manifold = nextCache.at(key);
        Body& a = bodies_[size_t(manifold.a)];
        Body* b = manifold.b >= 0 ? &bodies_[size_t(manifold.b)] : nullptr;
        const bool bInactive = !b || b->invMass <= 0 || b->sleeping;
        if (a.sleeping && bInactive) continue;
        for (int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex) {
            ContactPoint& point = manifold.points[size_t(pointIndex)];
            Vec3 impulse =
                point.normalImpulse * manifold.normal +
                point.tangentImpulse;

            // Only cached impulses are guarded. A persistent manifold may map
            // to a different corner after a tiny SAT feature change; reusing a
            // large old impulse there creates a nonphysical angular spike. New
            // impulses computed below remain unconstrained by this safeguard.
            double angularDelta = length(worldInvI(
                a, cross(point.point - a.p, -impulse)));
            if (b) angularDelta = std::max(
                angularDelta,
                length(worldInvI(*b, cross(point.point - b->p, impulse))));
            if (angularDelta > 0.55 && angularDelta > 1e-12) {
                const double scale = 0.55 / angularDelta;
                impulse *= scale;
                point.normalImpulse *= scale;
                point.tangentImpulse *= scale;
            }
            applyImpulse(a, -impulse, point.point, cfg_.wakeImpulse);
            if (b) applyImpulse(*b, impulse, point.point, cfg_.wakeImpulse);
        }
    }

    for (int iteration = 0; iteration < cfg_.velocityIterations; ++iteration) {
        for (uint64_t key : activeKeys) {
            ContactManifold& manifold = nextCache.at(key);
            Body& a = bodies_[size_t(manifold.a)];
            Body* b = manifold.b >= 0 ? &bodies_[size_t(manifold.b)] : nullptr;
            const bool bInactive = !b || b->invMass <= 0 || b->sleeping;
            if (a.sleeping && bInactive) continue;
            for (int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex) {
                ContactPoint& point = manifold.points[size_t(pointIndex)];
                const Vec3 velocityA = pointVelocity(a, point.point);
                const Vec3 velocityB = b ? pointVelocity(*b, point.point) : Vec3(0);
                Vec3 relativeVelocity = velocityB - velocityA;
                const double normalVelocity = dot(relativeVelocity, manifold.normal);

                double targetNormalVelocity = 0;
                if (point.penetration < 0) {
                    // Speculative CCD contact: allow approach only up to the
                    // remaining gap during this substep.
                    targetNormalVelocity = point.penetration / dt;
                } else if (point.penetration > 2.0 * cfg_.contactSlop) {
                    // Penetration is resolved by the split positional solve
                    // below. Injecting a positive separating velocity here
                    // converts overlap correction into kinetic energy and is
                    // the main cause of the long, slow pile-wide creep.
                    targetNormalVelocity = 0.0;
                }
                if (normalVelocity < -0.18 && cfg_.restitution > 0.0) {
                    targetNormalVelocity = std::max(
                        targetNormalVelocity,
                        -cfg_.restitution * normalVelocity);
                }

                const double normalMass = inverseEffectiveMass(
                    a,
                    b,
                    point.point,
                    manifold.normal);
                double deltaNormalImpulse =
                    (targetNormalVelocity - normalVelocity) / normalMass;
                const double oldNormalImpulse = point.normalImpulse;
                point.normalImpulse = std::max(
                    0.0,
                    oldNormalImpulse + deltaNormalImpulse);
                deltaNormalImpulse = point.normalImpulse - oldNormalImpulse;
                const Vec3 normalImpulse =
                    deltaNormalImpulse * manifold.normal;
                applyImpulse(a,
                             -normalImpulse,
                             point.point,
                             cfg_.wakeImpulse);
                if (b) {
                    applyImpulse(*b,
                                 normalImpulse,
                                 point.point,
                                 cfg_.wakeImpulse);
                }

                relativeVelocity =
                    (b ? pointVelocity(*b, point.point) : Vec3(0)) -
                    pointVelocity(a, point.point);
                const Vec3 tangentialVelocity = relativeVelocity -
                    dot(relativeVelocity, manifold.normal) * manifold.normal;
                const double tangentialSpeed = length(tangentialVelocity);
                if (tangentialSpeed > 1e-10) {
                    const Vec3 tangent = tangentialVelocity / tangentialSpeed;
                    const double tangentMass = inverseEffectiveMass(
                        a,
                        b,
                        point.point,
                        tangent);
                    const Vec3 deltaTangentImpulse =
                        (-tangentialSpeed / tangentMass) * tangent;
                    const Vec3 candidate =
                        point.tangentImpulse + deltaTangentImpulse;
                    const double frictionBlend = smootherstep(
                        0.006,
                        0.035,
                        tangentialSpeed);
                    const double friction = mix(
                        cfg_.staticFriction,
                        cfg_.dynamicFriction,
                        frictionBlend);
                    const double maximumTangent =
                        friction * point.normalImpulse;
                    Vec3 newTangent = candidate;
                    const double candidateLength = length(candidate);
                    if (candidateLength > maximumTangent && candidateLength > 1e-12) {
                        newTangent *= maximumTangent / candidateLength;
                    }
                    const Vec3 applied = newTangent - point.tangentImpulse;
                    point.tangentImpulse = newTangent;
                    applyImpulse(a, -applied, point.point, cfg_.wakeImpulse);
                    if (b) applyImpulse(*b, applied, point.point, cfg_.wakeImpulse);
                }
            }

            // Rolling and torsional resistance are angular impulses bounded by
            // the normal load and characteristic contact radius. No angular
            // velocity or per-impact clamp is used.
            double normalLoad = 0;
            for (int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex) {
                normalLoad += manifold.points[size_t(pointIndex)].normalImpulse;
            }
            if (normalLoad > 0) {
                const Vec3 relativeSpin = (b ? b->w : Vec3(0)) - a.w;
                const Vec3 torsional =
                    dot(relativeSpin, manifold.normal) * manifold.normal;
                const Vec3 rolling = relativeSpin - torsional;
                const double characteristicRadius = 0.5 *
                    std::min({a.half.x, a.half.y, a.half.z});
                auto solveAngular = [&](const Vec3& component,
                                        double frictionCoefficient) {
                    const double speed = length(component);
                    if (speed < 1e-10) return;
                    const Vec3 axis = component / speed;
                    double inverseAngularMass = 0.0;
                    if (!a.sleeping) {
                        inverseAngularMass +=
                            dot(axis, worldInvI(a, axis));
                    }
                    if (b && !b->sleeping) {
                        inverseAngularMass +=
                            dot(axis, worldInvI(*b, axis));
                    }
                    if (inverseAngularMass < 1e-12) return;
                    const double required = speed / inverseAngularMass;
                    const double limit = frictionCoefficient *
                        normalLoad * characteristicRadius;
                    const Vec3 angularImpulse =
                        -std::min(required, limit) * axis;
                    if (!a.sleeping) {
                        a.w -= worldInvI(a, angularImpulse);
                    }
                    if (b && !b->sleeping) {
                        b->w += worldInvI(*b, angularImpulse);
                    }
                };
                solveAngular(rolling, cfg_.rollingFriction);
                solveAngular(torsional, cfg_.torsionalFriction);
            }
        }
    }

    for (int i = 0; i < kModuleCount; ++i) {
        Body& body = bodies_[size_t(i)];
        if (body.sleeping) continue;

        // Continuous contact-localized loss approximates coating compliance,
        // acoustic radiation and micro-slip. It is governed only by the current
        // contact state and smoothly vanishes for free flight or energetic
        // impacts; there is no phase switch or end-of-shot viscosity change.
        if (body.contactCount > 0) {
            const double speed = length(body.v);
            const double spin = length(body.w);
            const double contactWeight = saturate(
                double(body.contactCount) / 4.0);
            const double lowLinear = 1.0 - smootherstep(0.020, 0.100, speed);
            const double lowAngular = 1.0 - smootherstep(0.20, 1.80, spin);
            const double linearRate = 0.75 * contactWeight * lowLinear;
            const double angularRate = 3.0 * contactWeight * lowAngular;
            body.v *= std::exp(-linearRate * dt);
            body.w *= std::exp(-angularRate * dt);
        }

        body.p += dt * body.v;
        body.q = integrateOrientation(body.q, body.w, dt);
        if (!finite(body.p) || !finite(body.v) || !finite(body.w)) {
            throw std::runtime_error("non-finite rigid-body state");
        }
    }

    // Nonlinear split-position solve. Positional correction is applied through
    // the same contact Jacobian as the velocity solve, including rotational
    // response. It changes pose only; it never fabricates separating velocity.
    auto applyOrientationCorrection = [&](Body& body, Vec3 delta) {
        if (body.sleeping || body.invMass <= 0.0) return;
        double angle = length(delta);
        if (angle < 1e-14) return;
        constexpr double maxAnglePerIteration = 0.045;
        if (angle > maxAnglePerIteration) {
            delta *= maxAnglePerIteration / angle;
            angle = maxAnglePerIteration;
        }
        body.q = normalized(quatAxisAngle(delta / angle, angle) * body.q);
    };

    for (int iteration = 0; iteration < cfg_.positionIterations; ++iteration) {
        for (uint64_t key : activeKeys) {
            ContactManifold& manifold = nextCache.at(key);
            Body& a = bodies_[size_t(manifold.a)];
            Body* b = manifold.b >= 0 ? &bodies_[size_t(manifold.b)] : nullptr;
            const bool bInactive = !b || b->invMass <= 0.0 || b->sleeping;
            if (a.sleeping && bInactive) continue;

            Vec3 normal = manifold.normal;
            Vec3 point{0, 0, 0};
            double penetration = 0.0;
            if (b) {
                const SatResult sat = queryRoundedBoxes(a, *b);
                if (!sat.overlap) continue;
                normal = sat.normal;
                const Vec3 pointA = supportRoundedBox(a, normal);
                const Vec3 pointB = supportRoundedBox(*b, -normal);
                point = 0.5 * (pointA + pointB);
                penetration = sat.penetration;
            } else {
                const Vec3 support = supportRoundedBox(a, {0, -1, 0});
                penetration = std::max(0.0, -support.y);
                normal = {0, -1, 0};
                point = {support.x, 0.0, support.z};
            }

            penetration = std::max(0.0, penetration - cfg_.contactSlop);
            if (penetration <= 0.0) continue;

            const double inverseMassA =
                (!a.sleeping && a.invMass > 0.0) ? a.invMass : 0.0;
            const double inverseMassB =
                (b && !b->sleeping && b->invMass > 0.0) ? b->invMass : 0.0;
            if (inverseMassA + inverseMassB <= 1e-12) continue;

            const Vec3 rA = point - a.p;
            const Vec3 rB = b ? point - b->p : Vec3(0);
            double inverseConstraintMass = inverseMassA + inverseMassB;
            if (inverseMassA > 0.0) {
                inverseConstraintMass += dot(
                    normal,
                    cross(worldInvI(a, cross(rA, normal)), rA));
            }
            if (b && inverseMassB > 0.0) {
                inverseConstraintMass += dot(
                    normal,
                    cross(worldInvI(*b, cross(rB, normal)), rB));
            }
            if (inverseConstraintMass <= 1e-12) continue;

            // Distribute the requested correction over the nonlinear passes.
            const double fraction = cfg_.positionPercent /
                double(std::max(1, cfg_.positionIterations));
            const double lambda = fraction * penetration /
                inverseConstraintMass;
            const Vec3 positionalImpulse = lambda * normal;

            if (inverseMassA > 0.0) {
                a.p -= inverseMassA * positionalImpulse;
                applyOrientationCorrection(
                    a,
                    -worldInvI(a, cross(rA, positionalImpulse)));
            }
            if (b && inverseMassB > 0.0) {
                b->p += inverseMassB * positionalImpulse;
                applyOrientationCorrection(
                    *b,
                    worldInvI(*b, cross(rB, positionalImpulse)));
            }
        }
    }

    const double resumeStabilizationRamp = stateLoaded_
        ? smootherstep(0.0, 0.35, time_)
        : 1.0;

    // Post-stabilization projects residual closing velocity out of the active
    // contact constraints. Unlike the previous realized-displacement rewrite,
    // this does not turn split positional corrections into actual body motion,
    // so buried dice do not migrate outward over many seconds.
    for (int stabilizationPass = 0; stabilizationPass < 2; ++stabilizationPass) {
        for (uint64_t key : activeKeys) {
            ContactManifold& manifold = nextCache.at(key);
            Body& a = bodies_[size_t(manifold.a)];
            Body* b = manifold.b >= 0 ? &bodies_[size_t(manifold.b)] : nullptr;
            const bool bInactive = !b || b->invMass <= 0.0 || b->sleeping;
            if (a.sleeping && bInactive) continue;

            for (int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex) {
                ContactPoint& point = manifold.points[size_t(pointIndex)];
                const Vec3 velocityA = pointVelocity(a, point.point);
                const Vec3 velocityB = b ? pointVelocity(*b, point.point)
                                         : Vec3(0);
                Vec3 relativeVelocity = velocityB - velocityA;
                const double normalVelocity =
                    dot(relativeVelocity, manifold.normal);

                if (normalVelocity < 0.0) {
                    const double mass = inverseEffectiveMass(
                        a, b, point.point, manifold.normal);
                    const Vec3 impulse =
                        (-normalVelocity / mass) * manifold.normal;
                    applyImpulse(a, -impulse, point.point, cfg_.wakeImpulse);
                    if (b) applyImpulse(*b, impulse, point.point, cfg_.wakeImpulse);
                }

                relativeVelocity =
                    (b ? pointVelocity(*b, point.point) : Vec3(0)) -
                    pointVelocity(a, point.point);
                const Vec3 tangentVelocity = relativeVelocity -
                    dot(relativeVelocity, manifold.normal) * manifold.normal;
                const double tangentSpeed = length(tangentVelocity);

                // Static-friction projection only in the quasi-static regime.
                // Faster sliding remains governed by the regular Coulomb solve.
                if (tangentSpeed > 1e-10 && tangentSpeed < 0.030) {
                    const Vec3 tangent = tangentVelocity / tangentSpeed;
                    const double mass = inverseEffectiveMass(
                        a, b, point.point, tangent);
                    Vec3 required = (-tangentSpeed / mass) * tangent;
                    const double supportImpulse = std::max(
                        point.normalImpulse,
                        0.20 * a.mass * cfg_.gravity * dt /
                            double(std::max(1, manifold.pointCount)));
                    const double limit = resumeStabilizationRamp *
                        cfg_.staticFriction * supportImpulse;
                    const double magnitude = length(required);
                    if (magnitude > limit && magnitude > 1e-12)
                        required *= limit / magnitude;
                    applyImpulse(a, -required, point.point, cfg_.wakeImpulse);
                    if (b) applyImpulse(*b, required, point.point, cfg_.wakeImpulse);
                }
            }
        }
    }

    manifoldCache_.swap(nextCache);

    if (cfg_.enableMotionSleep) {
        // Build a true load path rooted at the floor or the visible support.
        // A floating or unsupported island is never allowed to sleep, and a
        // moving neighbour wakes only the local contact island rather than the
        // entire pile.
        std::vector<std::vector<int>> neighbours{size_t(kModuleCount)};
        std::vector<int> persistentContacts(size_t(kModuleCount), 0);
        std::vector<uint8_t> directlyGrounded(size_t(kModuleCount), 0);

        for (uint64_t key : activeKeys) {
            const auto found = manifoldCache_.find(key);
            if (found == manifoldCache_.end()) continue;
            const ContactManifold& manifold = found->second;
            if (manifold.a < 0 || manifold.a >= kModuleCount) continue;

            // Geometric contact, rather than a fluctuating impulse threshold,
            // defines the support graph. Resting normal impulses can become
            // extremely small after warm starting; dropping those edges made
            // the support path flicker and woke hundreds of dice at once.
            if (manifold.pointCount <= 0) continue;

            if (manifold.b >= 0 && manifold.b < kModuleCount) {
                neighbours[size_t(manifold.a)].push_back(manifold.b);
                neighbours[size_t(manifold.b)].push_back(manifold.a);
                ++persistentContacts[size_t(manifold.a)];
                ++persistentContacts[size_t(manifold.b)];

                // Granular support is carried by a three-dimensional force
                // network, not only by contacts whose SAT normal happens to
                // point upward. Connectivity to a floor root is therefore
                // propagated through every persistent contact. A truly floating
                // clump has no floor-rooted path and remains dynamic.
            } else if (manifold.normal.y < -0.12) {
                // Floor and anchored trapdoor contacts are support roots.
                directlyGrounded[size_t(manifold.a)] = 1;
                ++persistentContacts[size_t(manifold.a)];
            }
        }

        std::vector<uint8_t> supportedPath(size_t(kModuleCount), 0);
        std::deque<int> supportQueue;
        for (int i = 0; i < kModuleCount; ++i) {
            if (directlyGrounded[size_t(i)]) {
                supportedPath[size_t(i)] = 1;
                supportQueue.push_back(i);
            }
        }
        while (!supportQueue.empty()) {
            const int current = supportQueue.front();
            supportQueue.pop_front();
            for (int neighbour : neighbours[size_t(current)]) {
                if (!supportedPath[size_t(neighbour)]) {
                    supportedPath[size_t(neighbour)] = 1;
                    supportQueue.push_back(neighbour);
                }
            }
        }

        // Sleeping is a local physical state, not a timeline phase. Once the
        // support has begun retracting, any grounded island that has remained
        // quasi-static may sleep even while distant dice are still falling.
        const bool mayEnterSleep = preRolling_ ||
            phase_ != AssemblyPhase::FixtureRetract;

        for (int i = 0; i < kModuleCount; ++i) {
            Body& body = bodies_[size_t(i)];
            const double ownLinear = length(body.v);
            const double ownAngular = length(body.w);
            double neighbourLinear = 0.0;
            double neighbourAngular = 0.0;
            for (int neighbour : neighbours[size_t(i)]) {
                neighbourLinear = std::max(
                    neighbourLinear,
                    length(bodies_[size_t(neighbour)].v));
                neighbourAngular = std::max(
                    neighbourAngular,
                    length(bodies_[size_t(neighbour)].w));
            }

            const bool supported = supportedPath[size_t(i)] != 0;
            const bool constrained = body.contactCount >= 1 &&
                persistentContacts[size_t(i)] > 0;
            const bool shallowContact =
                bodyMaxPenetration[size_t(i)] <= cfg_.sleepPenetration;

            // A dense granular core should become a static load-bearing
            // contact island much sooner than an exposed die that is still
            // free to rock or roll.  The previous one-size-fits-all sleep
            // threshold left hundreds of buried dice integrating tiny solver
            // corrections for more than a second, producing visible pile-wide
            // creep.  Deeply jammed dice use a higher capture threshold while
            // exposed surface dice retain strict ordinary rigid-body sleep.
            const bool deeplyJammed =
                persistentContacts[size_t(i)] >= 3 &&
                body.contactCount >= 6;
            const double linearSleepLimit = deeplyJammed
                ? 0.0100 : cfg_.sleepLinearSpeed;
            const double angularSleepLimit = deeplyJammed
                ? 0.38 : cfg_.sleepAngularSpeed;
            const double neighbourLinearLimit = deeplyJammed
                ? 0.022 : 0.010;
            const double neighbourAngularLimit = deeplyJammed
                ? 0.55 : 0.24;
            const double requiredQuietTime = deeplyJammed
                ? 0.085 : cfg_.sleepDelay;
            const bool localNeighbourhoodCalm =
                neighbourLinear < neighbourLinearLimit &&
                neighbourAngular < neighbourAngularLimit;
            const bool quiet = supported && constrained && shallowContact &&
                localNeighbourhoodCalm &&
                ownLinear < linearSleepLimit &&
                ownAngular < angularSleepLimit;

            if (body.sleeping) {
                // A sleeping die remains a local static member of the pile.
                // Explicit impact tests above wake it. The only non-impact wake
                // here is a genuinely unsupported, contact-free body that would
                // otherwise hover after its support moved away.
                const Vec3 lowestPoint =
                    supportRoundedBox(body, Vec3{0, -1, 0});
                const bool trulyAirborne =
                    body.contactCount == 0 && lowestPoint.y > 0.00035;
                if (trulyAirborne) wake(body);
                continue;
            }

            if (mayEnterSleep && quiet) {
                body.sleepTimer += dt;
            } else {
                // Brief manifold feature changes should not erase an entire
                // second of accumulated quiet time. Decay the timer smoothly.
                body.sleepTimer = std::max(0.0, body.sleepTimer - 2.0 * dt);
            }

            if (mayEnterSleep && body.sleepTimer >= requiredQuietTime) {
                body.sleeping = true;
                body.v = Vec3(0);
                body.w = Vec3(0);
                body.sleepReferenceMagneticForce = magneticForce[size_t(i)];
                body.sleepReferenceMagneticTorque = magneticTorque[size_t(i)];
            }
        }
    }

    time_ += dt;
}

std::vector<Snapshot> Simulator::capture() const {
    std::vector<Snapshot> snapshots;
    snapshots.reserve(bodies_.size());
    for (const Body& body : bodies_) {
        Snapshot snapshot;
        snapshot.p = body.p;
        snapshot.q = body.q;
        snapshot.v = body.v;
        snapshot.w = body.w;
        snapshot.half = body.half;
        snapshot.material = body.material;
        snapshot.id = body.id;
        snapshot.kind = int(body.kind);
        snapshot.moduleClass = body.moduleClass;
        snapshot.layer = body.layer;
        snapshot.slot = body.slot;
        snapshots.push_back(snapshot);
    }
    return snapshots;
}

FrameDiagnostics Simulator::measure() const {
    FrameDiagnostics diagnostics;
    diagnostics.time = time_;
    diagnostics.phase = phase_;
    diagnostics.globalMagnetization = globalMagnetization();
    diagnostics.meanMagnetization = globalMagnetization();
    diagnostics.permanentFieldFraction = 1.0;
    diagnostics.magneticWork = accumulatedMagneticWork_;
    diagnostics.closeMagneticPairs = lastCloseMagneticPairs_;
    diagnostics.snappedPolePairs = lastSnappedPolePairs_;
    diagnostics.contactManifolds = lastContactManifolds_;

    double meanHeight = 0;
    double totalSpeed = 0;
    double totalSpin = 0;
    double kineticEnergy = 0;
    int below = 0;
    int airborne = 0;
    for (int i = 0; i < kModuleCount; ++i) {
        const Body& body = bodies_[size_t(i)];
        const double speed = length(body.v);
        const double spin = length(body.w);
        meanHeight += body.p.y;
        totalSpeed += speed;
        totalSpin += spin;
        diagnostics.maxSpeed = std::max(diagnostics.maxSpeed, speed);
        diagnostics.maxSpin = std::max(diagnostics.maxSpin, spin);
        if (body.p.y < cfg_.settledHeight) ++below;
        if (body.p.y > body.half.y + 0.045) ++airborne;
        const Vec3 localW = body.q.inverseRotate(body.w);
        const Vec3 inertia{
            1.0 / body.invILocal.x,
            1.0 / body.invILocal.y,
            1.0 / body.invILocal.z
        };
        kineticEnergy += 0.5 * body.mass * lengthSquared(body.v) +
            0.5 * dot(inertia * localW, localW);
    }
    meanHeight /= double(kModuleCount);
    diagnostics.meanSpeed = totalSpeed / double(kModuleCount);
    diagnostics.meanSpin = totalSpin / double(kModuleCount);
    diagnostics.kineticEnergy = kineticEnergy;
    diagnostics.fractionBelow = double(below) / double(kModuleCount);
    diagnostics.airborneFraction = double(airborne) / double(kModuleCount);
    diagnostics.collapseFraction = saturate(
        (initialMeanHeight_ - meanHeight) /
        std::max(1e-6, initialMeanHeight_ - 0.105));

    std::vector<int> parent(kModuleCount);
    std::vector<int> size(kModuleCount, 1);
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](int x) {
        int r = x;
        while (parent[size_t(r)] != r) r = parent[size_t(r)];
        while (parent[size_t(x)] != x) {
            const int next = parent[size_t(x)];
            parent[size_t(x)] = r;
            x = next;
        }
        return r;
    };
    auto unite = [&](int a, int b) {
        int ra = root(a);
        int rb = root(b);
        if (ra == rb) return;
        if (size[size_t(ra)] < size[size_t(rb)]) std::swap(ra, rb);
        parent[size_t(rb)] = ra;
        size[size_t(ra)] += size[size_t(rb)];
    };

    std::unordered_map<int64_t, std::vector<int>, Int3Hash> grid;
    const double diagnosticCell = 0.050;
    grid.reserve(kModuleCount * 2);
    for (int i = 0; i < kModuleCount; ++i) {
        const Vec3& p = bodies_[size_t(i)].p;
        grid[cellKey(int(std::floor(p.x / diagnosticCell)),
                     int(std::floor(p.y / diagnosticCell)),
                     int(std::floor(p.z / diagnosticCell)))].push_back(i);
    }

    // Measure overlap from the post-solve transforms rather than reporting the
    // pre-correction manifold depth.  This is the release-quality penetration
    // diagnostic used by the validator.
    double measuredPenetration = 0;
    for (int i = 0; i < kModuleCount; ++i) {
        const Body& a = bodies_[size_t(i)];
        const int cx = int(std::floor(a.p.x / diagnosticCell));
        const int cy = int(std::floor(a.p.y / diagnosticCell));
        const int cz = int(std::floor(a.p.z / diagnosticCell));
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto it = grid.find(cellKey(cx + dx, cy + dy, cz + dz));
                    if (it == grid.end()) continue;
                    for (int j : it->second) {
                        if (j <= i) continue;
                        const Body& b = bodies_[size_t(j)];
                        const double contactRadius =
                            length(a.half) + length(b.half) + 0.002;
                        if (lengthSquared(b.p - a.p) >
                            contactRadius * contactRadius) continue;
                        const SatResult sat = queryRoundedBoxes(a, b);
                        if (sat.overlap) {
                            measuredPenetration = std::max(
                                measuredPenetration, sat.penetration);
                            unite(i, j);
                        } else if (sat.separation < 0.00022) {
                            unite(i, j);
                        }
                    }
                }
            }
        }
        const auto corners = coreCorners(a);
        for (const Vec3& corner : corners) {
            measuredPenetration = std::max(
                measuredPenetration,
                std::max(0.0, a.rounding - corner.y));
        }
        if (trapdoorBodyIndex_ >= 0) {
            const Body& door = bodies_[size_t(trapdoorBodyIndex_)];
            const double radius = length(a.half) + length(door.half) + 0.002;
            if (lengthSquared(door.p - a.p) <= radius * radius) {
                const SatResult sat = queryRoundedBoxes(a, door);
                if (sat.overlap) {
                    measuredPenetration = std::max(
                        measuredPenetration, sat.penetration);
                }
            }
        }
    }
    diagnostics.maxPenetration = measuredPenetration;

    double nearestSum = 0;
    for (int i = 0; i < kModuleCount; ++i) {
        const Body& a = bodies_[size_t(i)];
        double nearest = INF;
        const int cx = int(std::floor(a.p.x / diagnosticCell));
        const int cy = int(std::floor(a.p.y / diagnosticCell));
        const int cz = int(std::floor(a.p.z / diagnosticCell));
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto it = grid.find(cellKey(cx + dx, cy + dy, cz + dz));
                    if (it == grid.end()) continue;
                    for (int j : it->second) {
                        if (j == i) continue;
                        nearest = std::min(nearest, length(bodies_[size_t(j)].p - a.p));
                    }
                }
            }
        }
        if (std::isfinite(nearest)) nearestSum += nearest;
    }

    diagnostics.meanNearestNeighbor = nearestSum / double(kModuleCount);
    diagnostics.meanDipoleAlignment = 0;
    std::unordered_set<int> roots;
    for (int i = 0; i < kModuleCount; ++i) {
        const int r = root(i);
        roots.insert(r);
        diagnostics.largestCluster = std::max(
            diagnostics.largestCluster,
            size[size_t(r)]);
    }
    diagnostics.clusterCount = int(roots.size());
    return diagnostics;
}

SimulationResult Simulator::run() {
    if (!stateLoaded_) runPreRoll();
    SimulationResult result;
    result.frames.reserve(size_t(cfg_.frames));
    result.diagnostics.reserve(size_t(cfg_.frames));
    result.coils = coils_;
    const double frameDt = 1.0 / double(cfg_.fps);

    for (int frame = 0; frame < cfg_.frames; ++frame) {
        result.frames.push_back(capture());
        result.diagnostics.push_back(measure());
        FrameDiagnostics d = result.diagnostics.back();
        int sleepingCount = 0;
        for (int i = 0; i < kModuleCount; ++i) {
            if (bodies_[size_t(i)].sleeping) ++sleepingCount;
        }
        std::cout
            << "[physics] " << std::setw(3) << frame << "/" << cfg_.frames - 1
            << " t=" << std::fixed << std::setprecision(3) << d.time
            << " phase=" << phaseName(d.phase)
            << " collapse=" << std::setprecision(2) << d.collapseFraction
            << " speed=" << std::setprecision(3) << d.meanSpeed
            << " spin=" << std::setprecision(3) << d.meanSpin
            << " pen=" << std::setprecision(4) << d.maxPenetration
            << " sleep=" << sleepingCount
            << "      \r" << std::flush;

        // Continue integrating every recorded frame. Individual supported
        // contact islands may sleep through the local solver, but the scene is
        // never replaced by a duplicated frozen terminal frame.

        const int substeps = chooseSubsteps(frameDt);
        const double dt = frameDt / double(substeps);
        for (int substep = 0; substep < substeps; ++substep) step(dt);
    }
    std::cout << "\n";
    return result;
}

} // namespace tower
