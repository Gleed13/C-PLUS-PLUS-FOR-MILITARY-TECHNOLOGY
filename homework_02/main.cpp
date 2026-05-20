#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <optional>

const int MAX_STEPS = 10000; // maximum number of simulation steps
const float g = 9.81f;       // gravity acceleration in m/s^2
const float HIT_RADIUS_COEFFICIENT = 0.5f; // use smaller radius to increase chances of hitting the target

struct InputData
{
    float xd, yd, zd;       // drone coordinates
    float initialDir;       // initial direction of the drone in radians, 0 radians means facing north, pi/2 radians means facing west.
    float attackSpeed;      // meters per second
    float accelerationPath; // meters
    char ammoName[50];
    float arrayTimeStep;    // seconds
    float simTimeStep;      // seconds
    float hitRadius;        // damage radius — permissible hit error
    float angularSpeed;     // radians per second
    float turnThreshold;    // Threshold angle for stopping, radians
};
// for attackSpeed = 10 and accelerationPath = 10 we can calculate a = attackSpeed*attackSpeed / (2 * accelerationPath) = 5 m/s^2 - so we need 2 seconds to reach attack speed, and the drone will cover 10 meters during acceleration.
// decelerationPath = accelerationPath

enum DroneState
{
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

struct TargetsCoordinates
{
    // 5 targets, coordinates for each arrayTimeStep of simulation
    double targetXInTime[5][60];
    double targetYInTime[5][60];
};

struct OutputData
{
    int numberOfSteps;                  // N simulation total steps
    float droneCoords[MAX_STEPS * 2];   // coordinates at each step (x0 y0 x1 y1 ... xN-1 yN-1)
    float droneDirections[MAX_STEPS];   // (d0 d1 ... dN-1)
    DroneState droneStates[MAX_STEPS];  // (s0 s1 ... sN-1)
    int targetIndex[MAX_STEPS];         // index of the target at each step (t0 t1 ... tN-1)
};

struct Ammo
{
    const char* ammoType;
    float mass;
    float drag;
    float lift;
};

Ammo ammoTable[] =
{
    {"VOG-17",      0.35f, 0.07f, 0.0f},
    {"M67",         0.60f, 0.10f, 0.0f},
    {"RKG-3",       1.20f, 0.10f, 0.0f},
    {"GLIDING-VOG", 0.45f, 0.10f, 1.0f},
    {"GLIDING-RKG", 1.40f, 0.10f, 1.0f}
};

struct Vec2
{
    double x;
    double y;
};

struct DropPoint
{
    std::optional<Vec2> intermPoint; // intermediate point for too close targets
    Vec2 firePoint;                  // final firing point
};

InputData input;
TargetsCoordinates targetsCoordinates;
float mass, drag, lift;                 // mass in kg, drag and lift coefficients are dimensionless
float ffT;                              // projectile free-fall time in seconds
float hD;                               // projectile trajectory horizontal distance in meters
float acc;                              // drone acceleration in m/s^2
OutputData output;

Vec2 currentDroneCoordinates;
float currentDroneDirection;            // in radians, 0 means north, positive is counterclockwise
DroneState currentDroneState = STOPPED;
int currentTargetIndex = -1;            // index of the current target, -1 if no target
float currentSpeed = 0.0f;              // m/s

Vec2 currentTargetPosition;
DropPoint predictedDropPoint;
Vec2 predictedTargetPosition;

bool ReadInput(InputData* input)
{
    FILE* f = fopen("input.txt", "r");

    if (!f)
    {
        std::cout << "Error: input.txt file error" << std::endl;
        return false;
    }

    int scanned = fscanf(f, "%f %f %f %f %f %f %49s %f %f %f %f %f",
        &input->xd, &input->yd, &input->zd,
        &input->initialDir, &input->attackSpeed, &input->accelerationPath,
        input->ammoName,
        &input->arrayTimeStep, &input->simTimeStep, &input->hitRadius,
        &input->angularSpeed, &input->turnThreshold);

    fclose(f);

    if (scanned != 12)
    {
        std::cout << "Error: Invalid input format" << std::endl;
        return false;
    }

    currentDroneCoordinates = {input->xd, input->yd};
    currentDroneDirection = input->initialDir;

    return true;
}

bool LoadTargetsCoordinates(TargetsCoordinates* targets)
{
    std::string filePath = "targets.txt";
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        std::cout << "Error: Failed to open file: " << filePath << std::endl;
        return false;
    }

    int targetIndex;
    int stepIndex;
    // First 5 rows: X coordinates
    for (targetIndex = 0; targetIndex < 5; ++targetIndex)
    {
        for (stepIndex = 0; stepIndex < 60; ++stepIndex)
        {
            if (!(file >> targets->targetXInTime[targetIndex][stepIndex]))
            {
                std::cout << "Error: Failed to read X value for target "
                          << targetIndex << ", step " << stepIndex << std::endl;
                return false;
            }
        }
    }

    // Next 5 rows: Y coordinates
    for (targetIndex = 0; targetIndex < 5; ++targetIndex)
    {
        for (stepIndex = 0; stepIndex < 60; ++stepIndex)
        {
            if (!(file >> targets->targetYInTime[targetIndex][stepIndex]))
            {
                std::cout << "Error: Failed to read Y value for target "
                          << targetIndex << ", step " << stepIndex << std::endl;
                return false;
            }
        }
    }

    return true;
}

bool GetAmmoParams(const char* name, float* mass, float* drag, float* lift)
{
    int count = sizeof(ammoTable) / sizeof(ammoTable[0]);

    for (int i = 0; i < count; ++i)
    {
        if (strcmp(ammoTable[i].ammoType, name) == 0)
        {
            *mass = ammoTable[i].mass;
            *drag = ammoTable[i].drag;
            *lift = ammoTable[i].lift;
            return true;
        }
    }
    std::cout << "Error: Unknown ammo type: " << name << std::endl;

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

    std::cout << "Debug: Coefficients of cubic equation: a=" << a << ", b=" << b << ", c=" << c << std::endl;

    float p = -b*b / (3 * a*a);
    float q = (2 * b*b*b) / (27 * a*a*a) + c / a;

    std::cout << "Debug: Coefficients for depressed cubic: p=" << p << ", q=" << q << std::endl;
    if (p >= 0)
    {
        std::cout << "Error: p >= 0: No real roots" << std::endl;
        return false;
    }

    float phiArg = 3 * q / (2 * p) * sqrt(-3 / p);
    if (phiArg < -1.0f || phiArg > 1.0f)
    {
        std::cout << "Error: acos argument is out of bounds: " << phiArg << std::endl;
        return false;
    }

    float phi = acos(phiArg);
    std::cout << "Debug: phi (angle for cosine solution): " << phi << " radians" << std::endl;

    *t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);
    std::cout << "Debug: Projectile`s free-fall time: " << *t << " seconds" << std::endl;
    if (*t <= 0)
    {
        std::cout << "Error: Non-positive time to target: " << *t << " seconds" << std::endl;
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
    std::cout << "Debug: Projectile trajectory horizontal distance: " << *h << " meters" << std::endl;
    if (*h <= 0)
    {
        std::cout << "Error: Non-positive projectile trajectory horizontal distance: " << *h << " meters" << std::endl;
        return false;
    }

    return true;
}

bool CalculateAcceleration(const float& attackSpeed, const float& accelerationPath)
{
    if (accelerationPath <= 0)
    {
        std::cout << "Error: Non-positive acceleration path: " << accelerationPath << " meters" << std::endl;
        return false;
    }
    if (attackSpeed <= 0)
    {
        std::cout << "Error: Non-positive attack speed: " << attackSpeed << " m/s" << std::endl;
        return false;
    }
    acc = attackSpeed*attackSpeed / (2 * accelerationPath);
    
    return true;
}

float GetDistance(const Vec2& a, const Vec2& b)
{
    return sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

DropPoint CalculateDropPoint(
    const Vec2& dronePos,
    const Vec2& targetPos,
    const float& accelerationPath,
    const float& remainingAccelerationPath,
    const float& h,
    const float& droneDirection)
{
    Vec2 dronePosition = dronePos;
    DropPoint outputData;
    float distanceToTarget = GetDistance(dronePosition, targetPos);
    // std::cout << "Debug: Distance to target: " << distanceToTarget << " meters" << std::endl;
    if (distanceToTarget <= 0)
    {
        std::cout << "Debug: Non-positive distance to target: " << distanceToTarget << " meters" << std::endl;
        //calculate new drone position adding h distance in current direction of the drone
        dronePosition = {
            dronePos.x + h * cos(droneDirection),
            dronePos.y + h * sin(droneDirection)
        };
        distanceToTarget = GetDistance(dronePosition, targetPos);
    }

    float ratio = (distanceToTarget - h) / distanceToTarget;
    bool isTooCloseToTarget = h + remainingAccelerationPath - input.hitRadius * HIT_RADIUS_COEFFICIENT > distanceToTarget; // use half of hit radius as threshold to avoid unnecessary intermediate point creation
    if (isTooCloseToTarget)
    {
        outputData.intermPoint = {
            targetPos.x - (targetPos.x - dronePosition.x) * (h + accelerationPath) / distanceToTarget,
            targetPos.y - (targetPos.y - dronePosition.y) * (h + accelerationPath) / distanceToTarget
        };
    }
    else
    {
        outputData.intermPoint = std::nullopt;
    }
    outputData.firePoint = {
        targetPos.x - (targetPos.x - dronePosition.x) * ratio,
        targetPos.y - (targetPos.y - dronePosition.y) * ratio
    };

    return outputData;
}

double Length(const Vec2& v)
{
    return std::sqrt(v.x*v.x + v.y*v.y);
}

Vec2 Normalize(const Vec2& v)
{
    double len = Length(v);

    if (len == 0.0)
    {
        return {0.0, 0.0};
    }

    return {v.x / len, v.y / len};
}

// 1. direction angle in radians -> normalized direction vector
// 0 rad means north (+x), counterclockwise is positive
Vec2 DirectionToVector(const float& directionRad)
{
    return {
        std::cos(directionRad),
        std::sin(directionRad)
    };
}

// 2. direction vector -> direction angle in radians
// returns angle where 0 rad means north (+x)
float VectorToDirection(const Vec2& direction)
{
    return static_cast<float>(std::atan2(direction.y, direction.x));
}

// 3. angle from drone direction to target
// positive = target is to the left (counterclockwise)
// negative = target is to the right (clockwise)
float AngleToTargetRad(
    const Vec2& dronePos,
    const Vec2& targetPos,
    const Vec2& droneDir)
{
    Vec2 toTarget
    {
        targetPos.x - dronePos.x,
        targetPos.y - dronePos.y
    };

    Vec2 droneDirNorm = Normalize(droneDir);
    Vec2 toTargetNorm = Normalize(toTarget);

    if (Length(droneDirNorm) == 0.0 || Length(toTargetNorm) == 0.0)
    {
        return 0.0f;
    }

    double cross = droneDirNorm.x * toTargetNorm.y - droneDirNorm.y * toTargetNorm.x;
    double dot = droneDirNorm.x * toTargetNorm.x + droneDirNorm.y * toTargetNorm.y;

    return static_cast<float>(std::atan2(cross, dot));
}

Vec2 GetTargetPositionFromTime(const float& t, const int& targetId)
{
    int idx = static_cast<int>(std::floor(t / input.arrayTimeStep)) % 60;
    int next = (idx + 1) % 60;
    float frac = (t - idx * input.arrayTimeStep) / input.arrayTimeStep;
    Vec2 targetPos;
    targetPos.x = targetsCoordinates.targetXInTime[targetId][idx] + (targetsCoordinates.targetXInTime[targetId][next] - targetsCoordinates.targetXInTime[targetId][idx]) * frac;
    targetPos.y = targetsCoordinates.targetYInTime[targetId][idx] + (targetsCoordinates.targetYInTime[targetId][next] - targetsCoordinates.targetYInTime[targetId][idx]) * frac;

    return targetPos;
}

float GetDistanceBetweenPoints(const Vec2& a, const Vec2& b)
{
    return std::sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

float CalculateRemainingAccelerationPath(const float& currentSpeed)
{
    if (currentSpeed >= input.attackSpeed)
    {
        return 0.0f;
    }

    float speedRatio = currentSpeed / input.attackSpeed;
    float remainingPath = input.accelerationPath - input.accelerationPath * speedRatio * speedRatio;
    return remainingPath;
}

float CalculateRemainingDecelerationPath(const float& currentSpeed)
{
    if (currentSpeed == 0.0f)
    {
        return 0.0f;
    }

    float speedRatio = currentSpeed / input.attackSpeed;
    float remainingPath = input.accelerationPath * speedRatio * speedRatio;
    return remainingPath;
}

float CalculateTimeToTarget(const Vec2& targetPos)
{
    float decelerationTime = 0;
    float timeToIntermTurn = 0;
    float timeToIntermPoint = 0;
    float timeToTurn = 0;
    float accelerationTime = 0;
    float timeToTarget = 0;

    Vec2 droneCoords = currentDroneCoordinates;
    float droneDirection = currentDroneDirection;
    Vec2 droneDirVector = DirectionToVector(droneDirection);
    float droneSpeed = currentSpeed;

    float remainingAccPath = CalculateRemainingAccelerationPath(currentSpeed);
    float remainingDecPath = CalculateRemainingDecelerationPath(currentSpeed);
    DropPoint dropPoint = CalculateDropPoint(currentDroneCoordinates, targetPos, input.accelerationPath, remainingAccPath, hD, currentDroneDirection);

    if (dropPoint.intermPoint.has_value())
    {
        float angleToIntermPoint = AngleToTargetRad(droneCoords, dropPoint.intermPoint.value(), droneDirVector);
        if (std::abs(angleToIntermPoint) > input.turnThreshold)
        {
            decelerationTime = droneSpeed / acc; // time to decelerate to 0
            timeToIntermTurn = std::abs(angleToIntermPoint) / input.angularSpeed;
            droneDirection += angleToIntermPoint; // turn to intermediate point
            droneDirVector = DirectionToVector(droneDirection);
            float remainingDecPath = CalculateRemainingDecelerationPath(droneSpeed);
            // calculate new drone coordinates after deceleration using sin and cos
            droneCoords.x += remainingDecPath * cos(droneDirection);
            droneCoords.y += remainingDecPath * sin(droneDirection);
            remainingAccPath = input.accelerationPath;
            remainingDecPath = 0;
        }
        float distanceToIntermPoint = GetDistanceBetweenPoints(droneCoords, dropPoint.intermPoint.value()); // v^2 = v0^2 + 2*a*s
        if (distanceToIntermPoint <= remainingDecPath)
        {
            timeToIntermPoint = droneSpeed / acc; // time to decelerate to 0
        }
        else if (distanceToIntermPoint >= input.accelerationPath + remainingAccPath)
        {
            float distanceAtMaxSpeed = distanceToIntermPoint - remainingAccPath - input.accelerationPath;
            // accelerate, move at attack speed, decelerate
            timeToIntermPoint = (2 * input.attackSpeed - droneSpeed) / acc + distanceAtMaxSpeed / input.attackSpeed;
        }
        else
        {
            float accPathBeforeDecel = (distanceToIntermPoint - remainingDecPath) / 2;
            float peakSpeed = sqrt(droneSpeed * droneSpeed + 2 * acc * accPathBeforeDecel);
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
    if (std::abs(angleToTarget) > input.turnThreshold / (droneSpeed > 0 ? 1 : 2)) // use smaller threshold while speed is 0 for turning in place to avoid oscillations
    {
        timeToTurn = std::abs(angleToTarget) / input.angularSpeed;
    }
    accelerationTime = (input.attackSpeed - droneSpeed) / acc; // time to accelerate to attack speed from current speed
    remainingAccPath = CalculateRemainingAccelerationPath(droneSpeed);
    float distanceToTarget = GetDistanceBetweenPoints(droneCoords, targetPos);
    timeToTarget = (distanceToTarget - remainingAccPath) / input.attackSpeed;

    return decelerationTime + timeToIntermTurn + timeToIntermPoint + timeToTurn + accelerationTime + timeToTarget;
}

void RecordOutput(const int& currentStep,
    const float& xd,
    const float& yd,
    const float& direction,
    const DroneState& state,
    const int& targetIdx)
{
    output.droneCoords[currentStep * 2] = xd;
    output.droneCoords[currentStep * 2 + 1] = yd;
    output.droneDirections[currentStep] = direction;
    output.droneStates[currentStep] = state;
    output.targetIndex[currentStep] = targetIdx;
}

bool WriteOutput(const OutputData* outputData)
{
    std::ofstream file("simulation.txt");

    if (!file.is_open())
    {
        std::cout << " Error: Unable to open file" << std::endl;
        return false;
    }

    file << outputData->numberOfSteps << '\n';
    for (int i = 0; i < outputData->numberOfSteps; ++i)
    {
        if (i > 0) file << ' ';
        file << outputData->droneCoords[i * 2] << ' ' << outputData->droneCoords[i * 2 + 1];
    }
    file << '\n';
    for (int i = 0; i < outputData->numberOfSteps; ++i)
    {
        if (i > 0) file << ' ';
        file << outputData->droneDirections[i];
    }
    file << '\n';
    for (int i = 0; i < outputData->numberOfSteps; ++i)
    {
        if (i > 0) file << ' ';
        file << outputData->droneStates[i];
    }
    file << '\n';
    for (int i = 0; i < outputData->numberOfSteps; ++i)
    {
        if (i > 0) file << ' ';
        file << outputData->targetIndex[i];
    }

    return true;
}

void CalculateBestTimeToTarget(const float& currentTime)
{
    int targetIndex = -1;
    float bestTimeToTarget = std::numeric_limits<float>::max();
    Vec2 targetPosition = {0.0, 0.0};
    for (int i = 0; i < 5; ++i)
    {
        Vec2 targetPos = GetTargetPositionFromTime(currentTime, i);
        float timeToTarget = CalculateTimeToTarget(targetPos);
        if (timeToTarget < bestTimeToTarget)
        {
            targetIndex = i;
            bestTimeToTarget = timeToTarget;
            targetPosition = targetPos;
        }
    }

    currentTargetIndex = targetIndex;
    currentTargetPosition = targetPosition;
}

void CalculatePredictedDropPoint(const float& currentTime)
{
    float tPrev = currentTime - input.simTimeStep;
    Vec2 prevTargetPos = GetTargetPositionFromTime(tPrev, currentTargetIndex);
    Vec2 targetSpeed = {
        (currentTargetPosition.x - prevTargetPos.x) / input.simTimeStep,
        (currentTargetPosition.y - prevTargetPos.y) / input.simTimeStep
    };
    Vec2 predictedTargetPos = {
        currentTargetPosition.x + targetSpeed.x * ffT,
        currentTargetPosition.y + targetSpeed.y * ffT
    };

    float remainingAccPath = CalculateRemainingAccelerationPath(currentSpeed);
    predictedDropPoint = CalculateDropPoint(currentDroneCoordinates, predictedTargetPos, input.accelerationPath, remainingAccPath, hD, currentDroneDirection);
    predictedTargetPosition = predictedTargetPos;
}

void AccelerateDrone()
{
    currentSpeed += acc * input.simTimeStep;
    if (currentSpeed >= input.attackSpeed)
    {
        currentSpeed = input.attackSpeed;
        currentDroneState = MOVING;
    }
    else
    {
        currentDroneState = ACCELERATING;
    }
}

void DecelerateDrone()
{
    currentSpeed -= acc * input.simTimeStep;
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
    currentDroneCoordinates.x += currentSpeed * cos(currentDroneDirection) * input.simTimeStep;
    currentDroneCoordinates.y += currentSpeed * sin(currentDroneDirection) * input.simTimeStep;
}

void SimulateMovementToPredictedDropPoint()
{
    Vec2 targetPos;
    if (predictedDropPoint.intermPoint.has_value())
        targetPos = predictedDropPoint.intermPoint.value();
    else
        targetPos = predictedDropPoint.firePoint;
    float angleToTargetPos = AngleToTargetRad(currentDroneCoordinates, targetPos, DirectionToVector(currentDroneDirection));

    switch (currentDroneState)
    {
        case STOPPED:
        case TURNING:
            if (std::abs(angleToTargetPos) > input.turnThreshold / 2) // use smaller threshold for turning in place to avoid oscillations
            {
                currentDroneState = TURNING;
                int sign = (angleToTargetPos > 0) ? 1 : -1;
                currentDroneDirection += sign * std::min(input.angularSpeed * input.simTimeStep, std::abs(angleToTargetPos));
            }
            else
                AccelerateDrone();
            break;
        case ACCELERATING:
        case DECELERATING:
            if (std::abs(angleToTargetPos) > input.turnThreshold)
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
            if (std::abs(angleToTargetPos) > input.turnThreshold)
                DecelerateDrone();
            else if (predictedDropPoint.intermPoint.has_value())
            {
                float distanceToIntermPoint = GetDistanceBetweenPoints(currentDroneCoordinates, targetPos);
                if (distanceToIntermPoint <= input.accelerationPath)
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
    Vec2 predictedHitPosition = {
        currentDroneCoordinates.x + hD * cos(currentDroneDirection),
        currentDroneCoordinates.y + hD * sin(currentDroneDirection)
    };
    // check if target is within hit radius of dropped bomb
    float distanceToTarget = GetDistanceBetweenPoints(predictedHitPosition, predictedTargetPosition);

    return distanceToTarget <= input.hitRadius * HIT_RADIUS_COEFFICIENT; // use smaller radius to increase chances of hitting the target
}

int DroneSimulation()
{
    int currentStep = 0;

    while (currentStep < MAX_STEPS)
    {
        RecordOutput(currentStep, currentDroneCoordinates.x, currentDroneCoordinates.y, currentDroneDirection, currentDroneState, currentTargetIndex);

        if (currentStep == 0)
        {
            // we're chilling, cause we don't know target patterns yet, but we can already record initial state
            ++currentStep;
            continue;
        }

        float t = currentStep * input.simTimeStep;
        CalculateBestTimeToTarget(t);
        CalculatePredictedDropPoint(t);
        if (IsInFireRange())
        {
            output.numberOfSteps = currentStep + 1;
            return 0; // end simulation
        }
        SimulateMovementToPredictedDropPoint();

        ++currentStep;
    }

    output.numberOfSteps = MAX_STEPS;
    return 0;
}

int main()
{
    if (!ReadInput(&input))
    {
        return 1;
    }

    if (!LoadTargetsCoordinates(&targetsCoordinates))
    {
        return 1;
    }

    if (!GetAmmoParams(input.ammoName, &mass, &drag, &lift))
    {
        return 1;
    }

    if (!CalculateFreeFallTime(input.zd, input.attackSpeed, mass, drag, lift, &ffT))
    {
        return 1;
    }

    if (!CalculateHorizontalDistance(ffT, input.attackSpeed, mass, drag, lift, &hD))
    {
        return 1;
    }

    if (!CalculateAcceleration(input.attackSpeed, input.accelerationPath))
    {
        return 1;
    }

    int simulationResult = DroneSimulation();
    if (simulationResult != 0)
    {
        std::cout << "Error: Drone simulation failed" << std::endl;
        return 1;
    }

    if (!WriteOutput(&output))
    {
        return 1;
    }

    return 0;
}