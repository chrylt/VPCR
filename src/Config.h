#pragma once

#include <any>
#include <optional>
#include <string>
#include <unordered_map>

class Config final {
public:
    template <typename T>
    void Set(std::string option, T value)
    {
        config_[option] = value;
    }

    template <typename T>
    void SetDirty(std::string option, T value) const
    {
        dirtyConfig_[option] = std::any(value);
    }

    template <typename T>
    std::optional<T> Get(std::string option) const
    {
        if (!config_.contains(option)) {
            return std::nullopt;
        }

        try {
            return std::any_cast<T>(config_.at(option));
        } catch (const std::bad_any_cast&) {
            return std::nullopt;
        }
    }

    void ApplyDirty()
    {
        for (const auto& [option, value] : dirtyConfig_) {
            config_[option] = value;
        }
    }

private:
    mutable std::unordered_map<std::string, std::any> dirtyConfig_;
    std::unordered_map<std::string, std::any> config_;
};