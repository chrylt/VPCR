#include "ParseArgs.h"

#include <CLI11/CLI11.hpp>
#include <array>

#include "DefaultConfig.h"

Config ParseArgs(int argc, char **argv)
{
    auto config = GetDefaultConfig();
    CLI::App app{"Engine.exe"};

    app.add_option_function<std::string>(
           "--scene_file", [&config](const std::string& path) { config.Set("scenePath", path); },
           "Set the path to the scene file.")
        ->check(CLI::ExistingFile)
        ->type_name("PATH")
        ->required();

    app.add_option_function<std::vector<std::uint32_t>>(
           "--resolution",
           [&config](const std::vector<std::uint32_t>& resolution) { config.Set("resolution", resolution); },
           "Set the window resolution")
        ->expected(2);

    try {
        app.parse(argc, argv);
    } catch (const CLI::CallForHelp& e) {
        exit(app.exit(e));
    }

    return config;
}