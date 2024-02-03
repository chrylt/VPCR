#include "DefaultConfig.h"

#include <vector>

#include "Config.h"
#include "Utils.h"

Config GetDefaultConfig()
{
    Config config;

    std::vector<std::uint32_t> defaultResolution = {1000, 1000};
    config.Set("resolution", std::move(defaultResolution));
    config.Set("camera.moveSpeed", 1.f);
    config.Set("camera.lookSpeed", 0.1f);
    config.Set("LOD.culling", false);
    config.Set("LOD.acceleration", false);
    config.Set("LOD.colorBatch", false);
    config.Set("LOD.colorDepth", false);
    config.Set("LOD.level", 0);
    config.Set("LOD.maxTreeDepth", 0);
    config.Set("LOD.selection", 0.f);
    config.Set("LOD.defaultSelection", 0.f);
    config.Set("LOD.maxSelection", 0.f);
    config.Set("LOD.cullingFov", 70.f);
    config.Set("LOD.defaultCullingFov", 70.f);
    config.Set("VP.vis", false);
    config.Set("AA.currAAMode", AntiAliasingMode::Off);
    config.Set("AA.errorShow", false);
    config.Set("AA.preventOverflow", true);
    config.Set("AA.preventedOverflowVis", false);
    config.Set("DAA.visualizeDensityBuckets", false);
    config.Set("TPDAA.bucketIDToShow", 0);
    config.Set("OPDAA.bucketSize", (1000.0f / static_cast<float>(std::numeric_limits<int>::max())) * 100);

    return config;
}
