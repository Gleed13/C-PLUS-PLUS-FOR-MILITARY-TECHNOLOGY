struct InputData
{
    float xd, yd, zd;        // drone coordinates
    float xt, yt;            // target coordinates
    float attackSpeed;       // meters per second
    float accelerationPath;  // meters
    char ammoName[50];
};

struct OutputData
{
    bool isTooCloseToTarget;
    float intermXd;          // intermediate x-coordinate of the drone
    float intermYd;          // intermediate y-coordinate of the drone
    float fireX;             // x-coordinate where the projectile should be dropped
    float fireY;             // y-coordinate where the projectile should be dropped
};

struct Ammo
{
    const char* ammoType;
    float mass;
    float drag;
    float lift;
};

bool GetAmmoParams(const char* name, float* mass, float* drag, float* lift);

bool CalculateFreeFallTime(
    float zd,
    float attackSpeed,
    float mass,
    float drag,
    float lift,
    float* t);

bool calculateHorizontalDistance(
    float t,
    float attackSpeed,
    float mass,
    float drag,
    float lift,
    float* h);

bool calculateOutputData(
    float xd,
    float yd,
    float xt,
    float yt,
    float accelerationPath,
    float h,
    OutputData* outputData);

bool ComputeDropSolution(const InputData* input, OutputData* output);