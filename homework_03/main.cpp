#define _USE_MATH_DEFINES
#define ENABLE_LOG   1
#define ENABLE_DEBUG 0

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <optional>
#include <algorithm>
#include <limits>
#include "json.hpp"

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
  #define WARNING(msg) std::cout << "[WARNING] " << msg << std::endl
  #define ERROR(msg) std::cout << "[ERROR] " << msg << std::endl
#else
  #define LOG(msg)
  #define WARNING(msg)
  #define ERROR(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

using json = nlohmann::json;

const int MAX_STEPS = 10000;
const float g = 9.81f;
const float HIT_RADIUS_COEFFICIENT = 0.5f;

struct Coord
{
    float x;
    float y;

    Coord operator+(const Coord& other) const
    {
        return {x + other.x, y + other.y};
    }
    Coord operator-(const Coord& other) const
    {
        return {x - other.x, y - other.y};
    }
    Coord operator*(float s) const
    {
        return {x * s, y * s};
    }
    Coord operator/(float s) const
    {
        return {x / s, y / s};
    }
    bool operator==(const Coord& other) const
    {
        return x == other.x && y == other.y;
    }
};

float length(Coord c)
{
    return std::hypot(c.x, c.y);
}

Coord normalize(Coord c)
{
    float len = length(c);
    if (len == 0.0f)
        return {0.0f, 0.0f};

    return c / len;
}

float normalizeAngle(float angle)
{
    while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;

    return angle;
}

struct AmmoParams
{
    char name[32];
    float mass;
    float drag;
    float lift;
};

struct DroneConfig
{
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char ammoName[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep
{
    Coord pos;
    float direction;
    int state;
    int targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

enum DroneState
{
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

struct DropPoint
{
    std::optional<Coord> intermPoint;
    Coord firePoint;
};

DroneConfig config;
Coord** targets = nullptr;
int targetCount = 0;
int timeSteps = 0;
AmmoParams* ammo = nullptr;
int ammoCount = 0;
float mass, drag, lift;
float ffT, hD, acc;
SimStep* steps = nullptr;
int totalSteps = 0;

Coord currentDroneCoordinates;
float currentDroneDirection;
DroneState currentDroneState = STOPPED;
int currentTargetIndex = -1;
float currentSpeed = 0.0f;

Coord currentTargetPosition;
DropPoint predictedDropPoint;
Coord predictedTargetPosition;

bool LoadConfig()
{
    std::ifstream fin("config.json");
    if (!fin.is_open())
    {
        ERROR("config.json file error");
        return false;
    }

    json j;
    fin >> j;

    config.startPos.x = j["drone"]["position"]["x"];
    config.startPos.y = j["drone"]["position"]["y"];
    config.altitude = j["drone"]["altitude"];
    config.initialDir = j["drone"]["initialDirection"];
    config.attackSpeed = j["drone"]["attackSpeed"];
    config.accelPath = j["drone"]["accelerationPath"];
    config.angularSpeed = j["drone"]["angularSpeed"];
    config.turnThreshold = j["drone"]["turnThreshold"];
    const char* ammo = j["ammo"].get<std::string>().c_str();
    std::strncpy(config.ammoName, ammo, 31);
    config.ammoName[31] = '\0';
    config.simTimeStep = j["simulation"]["timeStep"];
    config.hitRadius = j["simulation"]["hitRadius"];
    config.arrayTimeStep = j["targetArrayTimeStep"];

    currentDroneCoordinates = config.startPos;
    currentDroneDirection = config.initialDir;

    LOG("Config loaded");

    return true;
}

bool LoadAmmo()
{
    std::ifstream fa("ammo.json");
    if (!fa.is_open())
    {
        ERROR("ammo.json file error");
        return false;
    }

    json ja;
    fa >> ja;
    ammoCount = (int)ja.size();
    ammo = new AmmoParams[ammoCount];
    for (int i = 0; i < ammoCount; i++)
    {
        std::strncpy(ammo[i].name, ja[i]["name"].get<std::string>().c_str(), 31);
        ammo[i].name[31] = '\0';
        ammo[i].mass = ja[i]["mass"];
        ammo[i].drag = ja[i]["drag"];
        ammo[i].lift = ja[i]["lift"];
    }
    LOG("Ammo loaded: " << ammoCount << " types");

    return true;
}

bool LoadTargets()
{
    std::ifstream ft("targets.json");
    if (!ft.is_open())
    {
        ERROR("targets.json file error");
        return false;
    }

    json jt;
    ft >> jt;
    targetCount = jt["targetCount"];
    timeSteps = jt["timeSteps"];

    targets = new Coord*[targetCount];
    for (int i = 0; i < targetCount; i++)
    {
        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++)
        {
            targets[i][j].x = jt["targets"][i]["positions"][j]["x"];
            targets[i][j].y = jt["targets"][i]["positions"][j]["y"];
        }
    }
    LOG("Targets loaded: " << targetCount << " targets, " << timeSteps << " steps");

    return true;
}

bool GetAmmoParams(const char* name)
{
    for (int i = 0; i < ammoCount; i++)
    {
        if (std::strcmp(ammo[i].name, name) == 0)
        {
            mass = ammo[i].mass;
            drag = ammo[i].drag;
            lift = ammo[i].lift;
            LOG("Ammo found: " << ammo[i].name);
            delete[] ammo; ammo = nullptr; // ammo data is no longer needed after extracting parameters
            return true;
        }
    }
    ERROR("Unknown ammo type: " << name);

    return false;
}

bool CalculateFreeFallTime(
    const float& zd,
    const float& attackSpeed,
    const float& mass,
    const float& drag,
    const float& lift,
    float* t)
{
    float a = drag * g * mass - 2 * drag*drag * lift * attackSpeed;
    float b = -3 * g * mass*mass + 3 * drag * lift * mass * attackSpeed;
    float c = 6 * mass*mass * zd;

    DEBUG("Coefficients of cubic equation: a=" << a << ", b=" << b << ", c=" << c);

    float p = -b*b / (3 * a*a);
    float q = (2 * b*b*b) / (27 * a*a*a) + c / a;

    DEBUG("Coefficients for depressed cubic: p=" << p << ", q=" << q);
    if (p >= 0)
    {
        ERROR("p >= 0: No real roots");
        return false;
    }

    float phiArg = 3 * q / (2 * p) * std::sqrt(-3.0f / p);
    if (phiArg < -1.0f || phiArg > 1.0f)
    {
        ERROR("acos argument is out of bounds: " << phiArg);
        return false;
    }

    float phi = std::acos(phiArg);
    DEBUG("phi (angle for cosine solution): " << phi << " radians");

    *t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * (float)M_PI) / 3) - b / (3 * a);
    DEBUG("Projectile's free-fall time: " << *t << " seconds");
    if (*t <= 0)
    {
        ERROR("Non-positive time to target: " << *t << " seconds");
        return false;
    }

    return true;
}

bool CalculateHorizontalDistance(
    const float& t,
    const float& attackSpeed,
    const float& mass,
    const float& drag,
    const float& lift,
    float* h)
{
    float t1 = attackSpeed * t;
    float t2 = -t*t * drag * attackSpeed / (2 * mass);
    float t3 = t*t*t * (6 * drag * g * lift * mass - 6 * drag*drag * (lift*lift - 1) * attackSpeed) / (36 * mass*mass);
    float t4 = t*t*t*t * (-6 * drag*drag * g * lift * (1 + lift*lift + lift*lift*lift*lift) * mass
                          + 3 * drag*drag*drag * lift*lift * (1 + lift*lift) * attackSpeed
                          + 6 * drag*drag*drag * lift*lift*lift*lift * (1 + lift*lift) * attackSpeed)
        / (36 * (1 + lift*lift)*(1 + lift*lift) * mass*mass*mass);
    float t5 = t*t*t*t*t * (3 * drag*drag*drag * g * lift*lift*lift * mass
                            - 3 * drag*drag*drag*drag * lift*lift * (1 + lift*lift) * attackSpeed)
        / (36 * (1 + lift*lift) * mass*mass*mass*mass);

    *h = t1 + t2 + t3 + t4 + t5;
    DEBUG("Projectile trajectory horizontal distance: " << *h << " meters");
    if (*h <= 0)
    {
        ERROR("Non-positive projectile trajectory horizontal distance: " << *h << " meters");
        return false;
    }

    return true;
}

bool CalculateAcceleration(const float& attackSpeed, const float& accelerationPath)
{
    if (accelerationPath <= 0)
    {
        ERROR("Non-positive acceleration path: " << accelerationPath << " meters");
        return false;
    }
    if (attackSpeed <= 0)
    {
        ERROR("Non-positive attack speed: " << attackSpeed << " m/s");
        return false;
    }
    acc = attackSpeed * attackSpeed / (2 * accelerationPath);

    return true;
}

float GetDistance(const Coord& a, const Coord& b)
{
    return length(b - a);
}

DropPoint CalculateDropPoint(
    const Coord& dronePos,
    const Coord& targetPos,
    const float& accelerationPath,
    const float& remainingAccelerationPath,
    const float& h,
    const float& droneDirection)
{
    Coord dronePosition = dronePos;
    DropPoint outputData;
    float distanceToTarget = GetDistance(dronePosition, targetPos);
    if (distanceToTarget <= 0)
    {
        DEBUG("Non-positive distance to target: " << distanceToTarget << " meters");
        dronePosition = dronePos + Coord{std::cos(droneDirection), std::sin(droneDirection)} * h;
        distanceToTarget = GetDistance(dronePosition, targetPos);
    }

    float ratio = (distanceToTarget - h) / distanceToTarget;
    bool isTooCloseToTarget = h + remainingAccelerationPath - config.hitRadius * HIT_RADIUS_COEFFICIENT > distanceToTarget;
    if (isTooCloseToTarget)
    {
        outputData.intermPoint = targetPos - (targetPos - dronePosition) * ((h + accelerationPath) / distanceToTarget);
    }
    else
    {
        outputData.intermPoint = std::nullopt;
    }
    outputData.firePoint = targetPos - (targetPos - dronePosition) * ratio;

    return outputData;
}

Coord DirectionToVector(const float& directionRad)
{
    return {std::cos(directionRad), std::sin(directionRad)};
}

float VectorToDirection(const Coord& direction)
{
    return std::atan2(direction.y, direction.x);
}

float AngleToTargetRad(
    const Coord& dronePos,
    const Coord& targetPos,
    const Coord& droneDir)
{
    Coord toTarget = targetPos - dronePos;
    Coord droneDirNorm = normalize(droneDir);
    Coord toTargetNorm = normalize(toTarget);

    if (length(droneDirNorm) == 0.0f || length(toTargetNorm) == 0.0f)
    {
        return 0.0f;
    }

    float cross = droneDirNorm.x * toTargetNorm.y - droneDirNorm.y * toTargetNorm.x;
    float dot = droneDirNorm.x * toTargetNorm.x + droneDirNorm.y * toTargetNorm.y;

    return std::atan2(cross, dot);
}

Coord GetTargetPositionFromTime(const float& t, const int& targetId)
{
    int idx = static_cast<int>(std::floor(t / config.arrayTimeStep)) % timeSteps;
    int next = (idx + 1) % timeSteps;
    float frac = (t - idx * config.arrayTimeStep) / config.arrayTimeStep;

    return targets[targetId][idx] + (targets[targetId][next] - targets[targetId][idx]) * frac;
}

float GetDistanceBetweenPoints(const Coord& a, const Coord& b)
{
    return length(b - a);
}

float CalculateRemainingAccelerationPath(const float& currentSpeed)
{
    if (currentSpeed >= config.attackSpeed)
        return 0.0f;

    float speedRatio = currentSpeed / config.attackSpeed;
    return config.accelPath - config.accelPath * speedRatio * speedRatio;
}

float CalculateRemainingDecelerationPath(const float& currentSpeed)
{
    if (currentSpeed == 0.0f)
        return 0.0f;

    float speedRatio = currentSpeed / config.attackSpeed;
    return config.accelPath * speedRatio * speedRatio;
}

float CalculateTimeToTarget(const Coord& targetPos)
{
    float decelerationTime = 0;
    float timeToIntermTurn = 0;
    float timeToIntermPoint = 0;
    float timeToTurn = 0;
    float accelerationTime = 0;
    float timeToTarget = 0;

    Coord droneCoords = currentDroneCoordinates;
    float droneDirection = currentDroneDirection;
    Coord droneDirVector = DirectionToVector(droneDirection);
    float droneSpeed = currentSpeed;

    float remainingAccPath = CalculateRemainingAccelerationPath(currentSpeed);
    float remainingDecPath = CalculateRemainingDecelerationPath(currentSpeed);
    DropPoint dropPoint = CalculateDropPoint(currentDroneCoordinates, targetPos, config.accelPath, remainingAccPath, hD, currentDroneDirection);

    if (dropPoint.intermPoint.has_value())
    {
        float angleToIntermPoint = AngleToTargetRad(droneCoords, dropPoint.intermPoint.value(), droneDirVector);
        if (std::abs(angleToIntermPoint) > config.turnThreshold)
        {
            decelerationTime = droneSpeed / acc; // time to decelerate to 0
            timeToIntermTurn = std::abs(angleToIntermPoint) / config.angularSpeed;
            droneDirection += angleToIntermPoint; // turn to intermediate point
            droneDirVector = DirectionToVector(droneDirection);
            float remainingDecPath = CalculateRemainingDecelerationPath(droneSpeed);
            // calculate new drone coordinates after deceleration using sin and cos
            droneCoords.x += remainingDecPath * std::cos(droneDirection);
            droneCoords.y += remainingDecPath * std::sin(droneDirection);
            remainingAccPath = config.accelPath;
            remainingDecPath = 0;
        }
        float distanceToIntermPoint = GetDistanceBetweenPoints(droneCoords, dropPoint.intermPoint.value()); // v^2 = v0^2 + 2*a*s
        if (distanceToIntermPoint <= remainingDecPath)
        {
            timeToIntermPoint = droneSpeed / acc; // time to decelerate to 0
        }
        else if (distanceToIntermPoint >= config.accelPath + remainingAccPath)
        {
            float distanceAtMaxSpeed = distanceToIntermPoint - remainingAccPath - config.accelPath;
            // accelerate, move at attack speed, decelerate
            timeToIntermPoint = (2 * config.attackSpeed - droneSpeed) / acc + distanceAtMaxSpeed / config.attackSpeed;
        }
        else
        {
            float accPathBeforeDecel = (distanceToIntermPoint - remainingDecPath) / 2;
            float peakSpeed = std::sqrt(droneSpeed * droneSpeed + 2 * acc * accPathBeforeDecel);
            // accelerate and decelerate without reaching attack speed
            // (v1 - v0) / a (acceleration time) + v1 / a (deceleration time) = (2 * v1 - v0) / a
            timeToIntermPoint = (2 * peakSpeed - droneSpeed) / acc;
        }
        // set drone coordinates and direction to intermediate point for next calculations
        droneCoords = dropPoint.intermPoint.value();
        droneDirection = angleToIntermPoint;
        droneDirVector = DirectionToVector(droneDirection);
        droneSpeed = 0; // we assume that drone will decelerate to 0 before turning and accelerating again
    }

    float angleToTarget = AngleToTargetRad(droneCoords, targetPos, droneDirVector);
    if (std::abs(angleToTarget) > config.turnThreshold / (droneSpeed > 0 ? 1 : 2)) // use smaller threshold while speed is 0 for turning in place to avoid oscillations
    {
        timeToTurn = std::abs(angleToTarget) / config.angularSpeed;
    }
    accelerationTime = (config.attackSpeed - droneSpeed) / acc; // time to accelerate to attack speed from current speed
    remainingAccPath = CalculateRemainingAccelerationPath(droneSpeed);
    float distanceToTarget = GetDistanceBetweenPoints(droneCoords, targetPos);
    timeToTarget = (distanceToTarget - remainingAccPath) / config.attackSpeed;

    return decelerationTime + timeToIntermTurn + timeToIntermPoint + timeToTurn + accelerationTime + timeToTarget;
}

bool WriteSimulation()
{
    std::ofstream fout("simulation.json");
    if (!fout.is_open())
    {
        ERROR("Unable to open simulation.json");
        return false;
    }

    json out;
    out["totalSteps"] = totalSteps;
    out["steps"] = json::array();
    for (int i = 0; i < totalSteps; i++)
    {
        json step;
        step["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
        step["direction"] = steps[i].direction;
        step["state"] = steps[i].state;
        step["targetIndex"] = steps[i].targetIdx;
        step["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
        step["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
        step["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
        out["steps"].push_back(step);
    }
    fout << out.dump(2);
    LOG("Simulation complete. Steps: " << totalSteps);

    return true;
}

void CalculateBestTimeToTarget(const float& currentTime)
{
    int bestIndex = -1;
    float bestTime = std::numeric_limits<float>::max();
    Coord bestPos = {0.0f, 0.0f};

    for (int i = 0; i < targetCount; i++)
    {
        Coord targetPos = GetTargetPositionFromTime(currentTime, i);
        float timeToTarget = CalculateTimeToTarget(targetPos);
        if (timeToTarget < bestTime)
        {
            bestIndex = i;
            bestTime = timeToTarget;
            bestPos = targetPos;
        }
    }
    currentTargetIndex = bestIndex;
    currentTargetPosition = bestPos;
}

void CalculatePredictedDropPoint(const float& currentTime)
{
    float tPrev = currentTime - config.simTimeStep;
    Coord prevTargetPos = GetTargetPositionFromTime(tPrev, currentTargetIndex);
    Coord targetSpeed = (currentTargetPosition - prevTargetPos) / config.simTimeStep;
    Coord predictedTargetPos = currentTargetPosition + targetSpeed * ffT;

    float remainingAccPath = CalculateRemainingAccelerationPath(currentSpeed);
    predictedDropPoint = CalculateDropPoint(currentDroneCoordinates, predictedTargetPos, config.accelPath, remainingAccPath, hD, currentDroneDirection);
    predictedTargetPosition = predictedTargetPos;
}

void AccelerateDrone()
{
    currentSpeed += acc * config.simTimeStep;
    if (currentSpeed >= config.attackSpeed)
    {
        currentSpeed = config.attackSpeed;
        currentDroneState = MOVING;
    }
    else
    {
        currentDroneState = ACCELERATING;
    }
}

void DecelerateDrone()
{
    currentSpeed -= acc * config.simTimeStep;
    if (currentSpeed <= 0)
    {
        currentSpeed = 0;
        currentDroneState = STOPPED;
    }
    else
    {
        currentDroneState = DECELERATING;
    }
}

void UpdateDroneCoordinates()
{
    currentDroneCoordinates.x += currentSpeed * std::cos(currentDroneDirection) * config.simTimeStep;
    currentDroneCoordinates.y += currentSpeed * std::sin(currentDroneDirection) * config.simTimeStep;
}

void SimulateMovementToPredictedDropPoint()
{
    Coord targetPos;
    if (predictedDropPoint.intermPoint.has_value())
        targetPos = predictedDropPoint.intermPoint.value();
    else
        targetPos = predictedDropPoint.firePoint;
    float angleToTargetPos = AngleToTargetRad(currentDroneCoordinates, targetPos, DirectionToVector(currentDroneDirection));

    switch (currentDroneState)
    {
        case STOPPED:
        case TURNING:
            if (std::abs(angleToTargetPos) > config.turnThreshold / 2) // use smaller threshold for turning in place to avoid oscillations
            {
                currentDroneState = TURNING;
                int sign = (angleToTargetPos > 0) ? 1 : -1;
                currentDroneDirection += sign * std::min(config.angularSpeed * config.simTimeStep, std::abs(angleToTargetPos));
            }
            else
                AccelerateDrone();
            break;
        case ACCELERATING:
        case DECELERATING:
            if (std::abs(angleToTargetPos) > config.turnThreshold)
                DecelerateDrone();
            else if (predictedDropPoint.intermPoint.has_value())
            {
                float distanceToIntermPoint = GetDistanceBetweenPoints(currentDroneCoordinates, targetPos);
                float remainingDecelPath = CalculateRemainingDecelerationPath(currentSpeed);
                if (distanceToIntermPoint <= remainingDecelPath)
                    DecelerateDrone();
                else
                    AccelerateDrone();
            }
            else
                AccelerateDrone();
            break;
        case MOVING:
            if (std::abs(angleToTargetPos) > config.turnThreshold)
                DecelerateDrone();
            else if (predictedDropPoint.intermPoint.has_value())
            {
                float distanceToIntermPoint = GetDistanceBetweenPoints(currentDroneCoordinates, targetPos);
                if (distanceToIntermPoint <= config.accelPath)
                    DecelerateDrone();
                else
                    currentDroneDirection += angleToTargetPos;
            }
            else
                currentDroneDirection += angleToTargetPos;
            break;
    }
    UpdateDroneCoordinates();
}

bool IsInFireRange()
{
    if (currentTargetIndex == -1)
        return false;
    if (currentDroneState != MOVING)
        return false;

    // get predicted hit position after ffT time
    Coord predictedHitPosition = currentDroneCoordinates + DirectionToVector(currentDroneDirection) * hD;
    // check if target is within hit radius of dropped bomb
    float distanceToTarget = GetDistanceBetweenPoints(predictedHitPosition, predictedTargetPosition);
    return distanceToTarget <= config.hitRadius * HIT_RADIUS_COEFFICIENT;
}

int DroneSimulation()
{
    int currentStep = 0;

    while (currentStep < MAX_STEPS)
    {
        steps[currentStep].pos = currentDroneCoordinates;
        steps[currentStep].direction = currentDroneDirection;
        steps[currentStep].state = currentDroneState;
        steps[currentStep].targetIdx = currentTargetIndex;
        steps[currentStep].aimPoint = currentDroneCoordinates + DirectionToVector(currentDroneDirection) * hD;

        if (currentStep == 0)
        {
            ++currentStep;
            continue;
        }

        float t = currentStep * config.simTimeStep;
        CalculateBestTimeToTarget(t);
        CalculatePredictedDropPoint(t);

        steps[currentStep].dropPoint = predictedDropPoint.firePoint;
        steps[currentStep].predictedTarget = predictedTargetPosition;

        if (IsInFireRange())
        {
            totalSteps = currentStep + 1;
            return 0;
        }
        SimulateMovementToPredictedDropPoint();
        ++currentStep;
    }

    totalSteps = MAX_STEPS;
    return 0;
}

int Simulation()
{
    if (!LoadConfig()) return 1;
    if (!LoadAmmo()) return 1;
    if (!LoadTargets()) return 1;

    if (!GetAmmoParams(config.ammoName)) return 1;
    if (!CalculateFreeFallTime(config.altitude, config.attackSpeed, mass, drag, lift, &ffT)) return 1;
    if (!CalculateHorizontalDistance(ffT, config.attackSpeed, mass, drag, lift, &hD)) return 1;
    if (!CalculateAcceleration(config.attackSpeed, config.accelPath)) return 1;

    steps = new SimStep[MAX_STEPS]();

    int simulationResult = DroneSimulation();
    if (simulationResult != 0)
        ERROR("Drone simulation failed");
    else
        WriteSimulation();

    return simulationResult;
}

int main()
{
    int simulationResult = Simulation();

    delete[] steps; steps = nullptr;
    for (int i = 0; i < targetCount; i++) delete[] targets[i];
    delete[] targets; targets = nullptr;
    delete[] ammo; ammo = nullptr;

    return simulationResult;
}
