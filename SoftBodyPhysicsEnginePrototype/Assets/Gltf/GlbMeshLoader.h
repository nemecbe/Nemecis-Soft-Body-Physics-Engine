#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "../../RenderEngine/RenderTypes.h"

namespace softphysics::assets
{
    // Initial importer for the prototype's static, single-root-node GLB meshes.
    std::optional<softphysics::renderengine::MeshData> load_glb_mesh(
        const std::filesystem::path& path,
        std::string& errorMessage);
}
