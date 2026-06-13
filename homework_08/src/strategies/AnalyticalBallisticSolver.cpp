#include <cmath>
#include <iostream>

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

    std::cout << "Debug: Coefficients of cubic equation: a=" << a << ", b=" << b << ", c=" << c << "\n";

    float p = -b * b / (3 * a * a);
    float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

    std::cout << "Debug: Coefficients for depressed cubic: p=" << p << ", q=" << q << "\n";
    if (p >= 0) {
        std::cout << "Error: p >= 0: No real roots" << std::endl;
        return false;
    }

    float phiArg = 3 * q / (2 * p) * sqrt(-3 / p);
    if (phiArg < -1.0f || phiArg > 1.0f) {
        std::cout << "Error: acos argument is out of bounds: " << phiArg << std::endl;
        return false;
    }

    float phi = acos(phiArg);
    std::cout << "Debug: phi (angle for cosine solution): " << phi << " radians" << std::endl;

    *t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);
    std::cout << "Debug: Projectile`s free-fall time: " << *t << " seconds" << std::endl;
    if (*t <= 0) {
        std::cout << "Error: Non-positive time to target: " << *t << " seconds" << std::endl;
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
    std::cout << "Debug: Projectile trajectory horizontal distance: " << *h << " meters" << std::endl;
    if (*h <= 0) {
        std::cout << "Error: Non-positive projectile trajectory horizontal distance: " << *h << " meters" << std::endl;
        return false;
    }

    return true;
}

bool AnalyticalBallisticSolver::tryCalculateDropPoint(
    float xd, float yd, float xt, float yt, float accelerationPath, float h, DropPoint* dropPoint)
{
    float distanceToTarget = sqrt((xt - xd) * (xt - xd) + (yt - yd) * (yt - yd));
    std::cout << "Debug: Distance to target: " << distanceToTarget << " meters" << std::endl;
    if (distanceToTarget <= 0) {
        std::cout << "Error: Non-positive distance to target: " << distanceToTarget << " meters" << std::endl;
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