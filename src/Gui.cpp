#include "Gui.h"

#include <imgui/imgui.h>

#include <vector>

void RenderGui(const Config& config)
{
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

        if (ImGui::RadioButton("Pairs", &currentWarpWideMode.value(), 1))
        {
            config.SetDirty("LOD.warpWideDeduplication", currentWarpWideMode.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Each pair of threads check together before sumbitting to the framebuffer");
        }
        ImGui::SameLine();

        if (ImGui::RadioButton("Full", &currentWarpWideMode.value(), 2))
        {
            config.SetDirty("LOD.warpWideDeduplication", currentWarpWideMode.value());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("The full warp check together before sumbitting to the framebuffer");
        }

        ImGui::TreePop();
    }

    ImGui::End();
}