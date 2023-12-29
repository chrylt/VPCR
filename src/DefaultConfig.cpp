#include "DefaultConfig.h"

#include "Config.h"

Config GetDefaultConfig()
{
    Config config;

    std::vector<std::uint32_t> defaultResolution = {1000, 1000};
    config.Set("resolution", std::move(defaultResolution));
    config.Set("camera.moveSpeed", 1.f);
    config.Set("camera.lookSpeed", 0.1f);

    return config;
}