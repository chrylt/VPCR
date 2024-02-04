#include "Gui.h"

#include <imgui/imgui.h>

#include <vector>

#include "Utils.h"

void RenderGui(const Config& config)
{
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowBgAlpha(0.7f);
    ImGui::Begin("State", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoTitleBar);
    ImGui::Text("Frame rate: %s FPS", config.Get<std::string>("TitleBar.fps").value().c_str());
    ImGui::Text("Batches   : %s/%s", config.Get<std::string>("TitleBar.DrawnBatches").value().c_str(),
                config.Get<std::string>("TitleBar.TotalBatches").value().c_str());
    ImGui::Text("Anti-Aliasing-Mode: %s", config.Get<std::string>("TitleBar.AAMode").value().c_str());
    ImGui::Text("Warp-Wide-Deduplication-Mode: %s", config.Get<std::string>("TitleBar.dedMode").value().c_str());

    ImGui::End();

    constexpr std::uint32_t settingsWidth = 260;
    const auto res = config.Get<std::vector<std::uint32_t>>("resolution").value();
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(res[0]) - settingsWidth, 0));
    ImGui::SetNextWindowSize(ImVec2(settingsWidth, static_cast<float>(res[1])));

    ImGui::Begin("Settings", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoMove);

    if (ImGui::TreeNodeEx("General", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto colorBatch = config.Get<bool>("LOD.colorBatch");
        if (ImGui::Checkbox("Color by Batch", &colorBatch.value())) {
            config.SetDirty("LOD.colorBatch", colorBatch.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Give each batch a unique color.");
        }
        auto vertexPrecisionVis = config.Get<bool>("VP.vis");
        if (ImGui::Checkbox("Color by Vertex Precision", &vertexPrecisionVis.value())) {
            config.SetDirty("VP.vis", vertexPrecisionVis.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Give each vertex precision a unique color."
                "\nRed : Low Precision (10 bit)"
                "\nGreen: Medium Precision (20 bit)"
                "\nBlue: High Precision (30 bit)");
        }

        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::TreeNodeEx("Culling", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto cullingEnabled = config.Get<bool>("LOD.culling");
        if (ImGui::Checkbox("Enable Frustum Culling", &cullingEnabled.value())) {
            config.SetDirty("LOD.culling", cullingEnabled.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Enables frustum culling which uses spheres which are set around batches.");
        }

        auto currentCullingFov = config.Get<float>("LOD.cullingFov");
        const auto maxCullingFov = config.Get<float>("LOD.defaultCullingFov");
        ImGui::Text("Culling FOV");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Change the culling frustum FOV without changing camera FOV.");
        }
        if (ImGui::SliderFloat("##Culling FOV", &currentCullingFov.value(), 0.f, maxCullingFov.value_or(0.f), "%.3f",
                               ImGuiSliderFlags_AlwaysClamp)) {
            config.SetDirty("LOD.cullingFov", currentCullingFov.value());
        }

        if (ImGui::Button("Reset Culling FOV")) {
            config.SetDirty("LOD.cullingFov", config.Get<float>("LOD.defaultCullingFov").value());
        }

        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::TreeNodeEx("LOD Acceleration", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto lodEnabled = config.Get<bool>("LOD.acceleration");
        if (ImGui::Checkbox("Enable LOD", &lodEnabled.value())) {
            config.SetDirty("LOD.acceleration", lodEnabled.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Enables lod acceleration structure.");
        }

        auto colorDepth = config.Get<bool>("LOD.colorDepth");
        if (ImGui::Checkbox("Color by Depth", &colorDepth.value())) {
            config.SetDirty("LOD.colorDepth", colorDepth.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Give each lod depth a unique color.");
        }

        auto currentTreeLayer = config.Get<int>("LOD.level");
        const auto maxTreeDepth = config.Get<int>("LOD.maxTreeDepth");
        ImGui::Text("Show LOD Level");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Show only the selected lod level. 0 means all levels.");
        }
        if (ImGui::SliderInt("##Show LOD Level", &currentTreeLayer.value(), 0, maxTreeDepth.value_or(0), "%d",
                             ImGuiSliderFlags_AlwaysClamp)) {
            config.SetDirty("LOD.level", currentTreeLayer.value());
        }

        auto currentLODSelection = config.Get<float>("LOD.selection");
        const auto maxLODSelection = config.Get<float>("LOD.maxSelection");
        ImGui::Text("LOD Selection Criteria");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Selects LOD levels that fill at least as many pixels on the screen as set by the criteria.");
        }
        if (ImGui::SliderFloat("##LOD Selection Criteria", &currentLODSelection.value(), 0.f,
                               maxLODSelection.value_or(0.f), "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
            config.SetDirty("LOD.selection", currentLODSelection.value());
        }

        if (ImGui::Button("Reset selection")) {
            config.SetDirty("LOD.selection", config.Get<float>("LOD.defaultSelection").value());
        }

        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::TreeNodeEx("Warp Wide Deduplication", ImGuiTreeNodeFlags_DefaultOpen)) {
        auto currentWarpWideMode = config.Get<int>("LOD.warpWideDeduplication");
        if (ImGui::RadioButton("None", &currentWarpWideMode.value(), 0)) {
            config.SetDirty("LOD.warpWideDeduplication", currentWarpWideMode.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Each thread submit to the framebuffer");
        }
        ImGui::SameLine();

        if (ImGui::RadioButton("Pairs", &currentWarpWideMode.value(), 1)) {
            config.SetDirty("LOD.warpWideDeduplication", currentWarpWideMode.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Each pair of threads check together before sumbitting to the framebuffer");
        }
        ImGui::SameLine();

        if (ImGui::RadioButton("Full", &currentWarpWideMode.value(), 2)) {
            config.SetDirty("LOD.warpWideDeduplication", currentWarpWideMode.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("The full warp check together before sumbitting to the framebuffer");
        }

        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (ImGui::TreeNodeEx("Anti-Aliasing", ImGuiTreeNodeFlags_DefaultOpen)) {
        static const char *items[]{"None", "Two-Pass Percentage", "One-Pass Density", "Two-Pass Density"};
        int currMode = config.Get<AntiAliasingMode>("AA.currAAMode").value();
        if (ImGui::Combo("##Anti-Aliasing Mode", &currMode, items, IM_ARRAYSIZE(items))) {
            config.SetDirty("AA.currAAMode", static_cast<AntiAliasingMode>(currMode));
        }

        auto errorShow = config.Get<bool>("AA.errorShow");
        if (ImGui::Checkbox("Show Error Color", &errorShow.value())) {
            config.SetDirty("AA.errorShow", errorShow.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Colors areas according to error codes. "
                "\nMagenta: Color Accumulation Overflow"
                "\n(OPDAA) Red: Failed Insert, but reached End of Linked List"
                "\n(OPDAA) Green: Linked List Insert Timeout"
                "\n(OPDAA) Blue: Filled Bucket Count at Limit");
        }

        auto preventedOverflowVis = config.Get<bool>("AA.preventedOverflowVis");
        if (ImGui::Checkbox("Color Accumulation Overflow", &preventedOverflowVis.value())) {
            config.SetDirty("AA.preventedOverflowVis", preventedOverflowVis.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Visualizes accumulation overflow prevention.");
        }

        auto depthPerc = config.Get<float>("TPAA.depthPerc");
        ImGui::Text("(TPAA) Depth Percentage");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Points chosen for accumulation have to be within the range of this depth percentage.");
        }
        if (ImGui::SliderFloat("##(TPAA) Depth Percentage for Accumulation", &depthPerc.value(), 0, 1, "%.5f",
                               ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic)) {
            config.SetDirty("TPAA.depthPerc", depthPerc.value());
        }

        if (ImGui::TreeNodeEx("Density-Based Methods", ImGuiTreeNodeFlags_DefaultOpen)) {
            auto preventOverflow = config.Get<bool>("AA.preventOverflow");
            if (ImGui::Checkbox("Enable Overflow Prevention of Accumulation", &preventOverflow.value())) {
                config.SetDirty("AA.preventOverflow", preventOverflow.value());
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Toggles accumulation overflow prevention by discarding buckets.");
            }

            auto visualizeDensityBuckets = config.Get<bool>("DAA.visualizeDensityBuckets");
            if (ImGui::Checkbox("Visualize Histogram Buckets", &visualizeDensityBuckets.value())) {
                config.SetDirty("DAA.visualizeDensityBuckets", visualizeDensityBuckets.value());
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Visualizes buckets of the histogram.\n(OPDAA) Randomly assigns color to bucketID\n(TPDAA) Only "
                    "renders points in bucket specified by bucketID below.");
            }

            auto bucketSize = config.Get<float>("OPDAA.bucketSize");
            ImGui::Text("(OPDAA) Set Bucket Size");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Set the world space bucket size for the one-pass density-based anti-aliasing approach.");
            }
            if (ImGui::SliderFloat("##(OPDAA) Set Bucket Size", &bucketSize.value(), 0.0000001, 0.01, "%.10f",
                                   ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic)) {
                config.SetDirty("OPDAA.bucketSize", bucketSize.value());
            }

            auto bucketIDToVis = config.Get<int>("TPDAA.bucketIDToShow");
            ImGui::Text("(TPDAA) Visualize by BucketID");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Only render the buckets that correspond to the selected bucketID");
            }
            if (ImGui::SliderInt("##Visualize by BucketID", &bucketIDToVis.value(), 0, BUCKET_COUNT_TPDAA - 1, "%d",
                                 ImGuiSliderFlags_AlwaysClamp)) {
                config.SetDirty("TPDAA.bucketIDToShow", bucketIDToVis.value());
            }

            ImGui::TreePop();
        }

        ImGui::TreePop();
    }

    ImGui::End();
}