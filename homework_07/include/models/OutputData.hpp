#pragma once

struct OutputData
{
    bool isTooCloseToTarget;
    float intermXd;          // intermediate x-coordinate of the drone
    float intermYd;          // intermediate y-coordinate of the drone
    float fireX;             // x-coordinate where the projectile should be dropped
    float fireY;             // y-coordinate where the projectile should be dropped
};