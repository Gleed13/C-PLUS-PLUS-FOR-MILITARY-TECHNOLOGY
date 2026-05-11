#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <cstring>

#include "ballistics.hpp"

const float g = 9.81f;  // gravity acceleration in m/s^2

Ammo ammoTable[] = {{"VOG-17", 0.35f, 0.07f, 0.0f},
                    {"M67", 0.60f, 0.10f, 0.0f},
                    {"RKG-3", 1.20f, 0.10f, 0.0f},
                    {"GLIDING-VOG", 0.45f, 0.10f, 1.0f},
                    {"GLIDING-RKG", 1.40f, 0.10f, 1.0f}};

bool GetAmmoParams(const char* name, float* mass, float* drag, float* lift)
{
  int count = sizeof(ammoTable) / sizeof(ammoTable[0]);

  for (int i = 0; i < count; i++) {
    if (strcmp(ammoTable[i].ammoType, name) == 0) {
      *mass = ammoTable[i].mass;
      *drag = ammoTable[i].drag;
      *lift = ammoTable[i].lift;
      return true;
    }
  }

  return false;
}

bool CalculateFreeFallTime(float zd, float attackSpeed, float mass, float drag, float lift, float* t)
{
  float a = drag * g * mass - 2 * drag * drag * lift * attackSpeed;
  float b = -3 * g * mass * mass + 3 * drag * lift * mass * attackSpeed;
  float c = 6 * mass * mass * zd;

  std::cout << "Debug: Coefficients of cubic equation: a=" << a << ", b=" << b << ", c=" << c << std::endl;

  float p = -b * b / (3 * a * a);
  float q = (2 * b * b * b) / (27 * a * a * a) + c / a;

  std::cout << "Debug: Coefficients for depressed cubic: p=" << p << ", q=" << q << std::endl;
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

bool calculateHorizontalDistance(float t, float attackSpeed, float mass, float drag, float lift, float* h)
{
  float t1 = attackSpeed * t;
  float t2 = -t * t * drag * attackSpeed / (2 * mass);
  float t3 = t * t * t * (6 * drag * g * lift * mass - 6 * drag * drag * (lift * lift - 1) * attackSpeed) / (36 * mass * mass);
  float t4 = t * t * t * t *
             (-6 * drag * drag * g * lift * (1 + lift * lift + lift * lift * lift * lift) * mass +
              3 * drag * drag * drag * lift * lift * (1 + lift * lift) * attackSpeed +
              6 * drag * drag * drag * lift * lift * lift * lift * (1 + lift * lift) * attackSpeed) /
             (36 * (1 + lift * lift) * (1 + lift * lift) * mass * mass * mass);
  float t5 = t * t * t * t * t *
             (3 * drag * drag * drag * g * lift * lift * lift * mass -
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

bool calculateOutputData(float xd, float yd, float xt, float yt, float accelerationPath, float h, OutputData* outputData)
{
  float distanceToTarget = sqrt((xt - xd) * (xt - xd) + (yt - yd) * (yt - yd));
  std::cout << "Debug: Distance to target: " << distanceToTarget << " meters" << std::endl;
  if (distanceToTarget <= 0) {
    std::cout << "Error: Non-positive distance to target: " << distanceToTarget << " meters" << std::endl;
    return false;
  }

  float ratio = (distanceToTarget - h) / distanceToTarget;
  outputData->isTooCloseToTarget = h + accelerationPath > distanceToTarget;
  if (outputData->isTooCloseToTarget) {
    outputData->intermXd = xt - (xt - xd) * (h + accelerationPath) / distanceToTarget;
    outputData->intermYd = yt - (yt - yd) * (h + accelerationPath) / distanceToTarget;
  }
  else {
    outputData->intermXd = 0.0f;
    outputData->intermYd = 0.0f;
  }
  outputData->fireX = xd + (xt - xd) * ratio;
  outputData->fireY = yd + (yt - yd) * ratio;

  return true;
}

bool ComputeDropSolution(const InputData* input, OutputData* output)
{
  float mass, drag, lift;  // mass in kg, drag and lift coefficients are dimensionless
  float t;                 // projectile free-fall time in seconds
  float h;                 // projectile trajectory horizontal distance in meters

  if (!GetAmmoParams(input->ammoName, &mass, &drag, &lift)) {
    std::cout << "Error: Unknown ammo type: " << input->ammoName << std::endl;
    return 1;
  }

  if (!CalculateFreeFallTime(input->zd, input->attackSpeed, mass, drag, lift, &t)) {
    return 1;
  }

  if (!calculateHorizontalDistance(t, input->attackSpeed, mass, drag, lift, &h)) {
    return 1;
  }

  if (!calculateOutputData(input->xd, input->yd, input->xt, input->yt, input->accelerationPath, h, output)) {
    return 1;
  }

  return 0;
}