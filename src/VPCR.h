#pragma once
#include <memory>

#include "Config.h"

class VPCR {
public:
    virtual void Run() = 0;
};

std::unique_ptr<VPCR> CreateVPCR(Config config);