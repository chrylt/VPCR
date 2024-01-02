#include "ParseArgs.h"
#include "VPCR.h"

int main(int argc, char **argv)
{
    auto config = ParseArgs(argc, argv);

    auto vpcr = CreateVPCR(std::move(config));

    vpcr->Run();

    return 0;
}
