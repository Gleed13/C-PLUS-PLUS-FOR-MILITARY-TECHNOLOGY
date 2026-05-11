#include <iostream>

#include "ballistics.hpp"

bool ReadInput(InputData* input)
{
    FILE* f = fopen("input.txt", "r");

    if (!f)
    {
        std::cout << "Error: File error" << std::endl;
        return false;
    }

    int scanned = fscanf(f, "%f %f %f %f %f %f %f %49s",
        &input->xd, &input->yd, &input->zd,
        &input->xt, &input->yt,
        &input->attackSpeed, &input->accelerationPath,
        input->ammoName);

    fclose(f);

    if (scanned != 8)
    {
        std::cout << "Error: Invalid input format" << std::endl;
        return false;
    }

    return true;
}

bool writeOutput(const OutputData* outputData)
{
    FILE* out = fopen("output.txt", "w");

    if (!out)
    {
        std::cout << "Error: Cannot create output.txt" << std::endl;
        return false;
    }

    if (outputData->isTooCloseToTarget)
    {
        fprintf(out, "%.3f %.3f %.3f %.3f\n",
            outputData->intermXd,
            outputData->intermYd,
            outputData->fireX,
            outputData->fireY);
    }
    else
    {
        fprintf(out, "%.3f %.3f\n",
            outputData->fireX,
            outputData->fireY);
    }

    fclose(out);
    return true;
}

int main()
{
    InputData input;
    OutputData output;

    if (!ReadInput(&input))
    {
        return 1;
    }

    if (!ComputeDropSolution(&input, &output))
    {
        return 1;
    }

    if (!writeOutput(&output))
    {
        return 1;
    }

    return 0;
}