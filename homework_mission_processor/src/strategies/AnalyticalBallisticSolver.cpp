#include <cmath>

#include "features/Logging.hpp"
#include "strategies/AnalyticalBallisticSolver.hpp"

bool AnalyticalBallisticSolver::trySolve(const DroneConfig* droneConfig, const Coord* targetPos, const Ammo* ammo, DropPoint* dropPoint)
{
    float t = NAN;
    float h = NAN;
    if (!tryCalculateFreeFallTime(droneConfig->altitude, droneConfig->attackSpeed, ammo->mass, ammo->drag, ammo->lift, &t)) {
        return false;
    }
    if (!tryCalculateHorizontalDistance(t, droneConfig->attackSpeed, ammo->mass, ammo->drag, ammo->lift, &h)) {
        return false;
    }
    if (!tryCalculateDropPoint(
            droneConfig->startPos.x, droneConfig->startPos.y, targetPos->x, targetPos->y, droneConfig->accelPath, h, dropPoint)) {
        return false;
    }
    return true;
}

bool AnalyticalBallisticSolver::tryCalculateFreeFallTime(float zd, float attackSpeed, float mass, float drag, float lift, float* t)
{
    float a = drag * kGravity * mass - 2 * drag * drag * lift * attackSpeed;
    float b = -3 * kGravity * mass * mass + 3 * drag * lift * mass * attackSpeed;
    float c = 6 * mass * mass * zd;

    DEBUG("Coefficients of cubic equation: a=" << a << ", b=" << b << ", c=" << c);

    float p = -b * b / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

    DEBUG("Coefficients for depressed cubic: p=" << p << ", q=" << q);
    if (p >= 0) {
        ERROR("p >= 0: No real roots");
        return false;
    }

    float phiArg = 3 * q / (2 * p) * sqrt(-3 / p);
    if (phiArg < -1.0f || phiArg > 1.0f) {
        ERROR("acos argument is out of bounds: " << phiArg);
        return false;
    }

    float phi = acos(phiArg);
    DEBUG("phi (angle for cosine solution): " << phi << " radians");

    *t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);
    DEBUG("Projectile's free-fall time: " << *t << " seconds");
    if (*t <= 0) {
        ERROR("Non-positive time to target: " << *t << " seconds");
        return false;
    }

    return true;
}

bool AnalyticalBallisticSolver::tryCalculateHorizontalDistance(float t, float attackSpeed, float mass, float drag, float lift, float* h)
{
    float t1 = attackSpeed * t;
    float t2 = -t * t * drag * attackSpeed / (2 * mass);
    float t3 = t * t * t * (6 * drag * kGravity * lift * mass - 6 * drag * drag * (lift * lift - 1) * attackSpeed) / (36 * mass * mass);
    float t4 = t * t * t * t *
               (-6 * drag * drag * kGravity * lift * (1 + lift * lift + lift * lift * lift * lift) * mass +
                3 * drag * drag * drag * kGravity * lift * lift * (1 + lift * lift) * attackSpeed +
                6 * drag * drag * drag * kGravity * lift * lift * lift * lift * (1 + lift * lift) * attackSpeed) /
               (36 * (1 + lift * lift) * (1 + lift * lift) * mass * mass * mass);
    float t5 = t * t * t * t * t *
               (3 * drag * drag * drag * kGravity * lift * lift * lift * mass -
                3 * drag * drag * drag * drag * lift * lift * (1 + lift * lift) * attackSpeed) /
               (36 * (1 + lift * lift) * mass * mass * mass * mass);

    *h = t1 + t2 + t3 + t4 + t5;
    DEBUG("Projectile trajectory horizontal distance: " << *h << " meters");
    if (*h <= 0) {
        ERROR("Non-positive projectile trajectory horizontal distance: " << *h << " meters");
        return false;
    }

    return true;
}

bool AnalyticalBallisticSolver::tryCalculateDropPoint(
    float xd, float yd, float xt, float yt, float accelerationPath, float h, DropPoint* dropPoint)
{
    float distanceToTarget = sqrt((xt - xd) * (xt - xd) + (yt - yd) * (yt - yd));
    DEBUG("Distance to target: " << distanceToTarget << " meters");
    if (distanceToTarget <= 0) {
        ERROR("Non-positive distance to target: " << distanceToTarget << " meters");
        return false;
    }

    float ratio = (distanceToTarget - h) / distanceToTarget;
    bool isTooCloseToTarget = h + accelerationPath > distanceToTarget;
    if (isTooCloseToTarget) {
        dropPoint->intermPoint =
            Coord{xt - (xt - xd) * (h + accelerationPath) / distanceToTarget, yt - (yt - yd) * (h + accelerationPath) / distanceToTarget};
    }
    else {
        dropPoint->intermPoint = std::nullopt;
    }
    dropPoint->firePoint = Coord{xd + (xt - xd) * ratio, yd + (yt - yd) * ratio};

    return true;
}
