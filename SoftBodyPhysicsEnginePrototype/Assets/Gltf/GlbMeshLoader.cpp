#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#include "GlbMeshLoader.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

namespace
{
    using softphysics::renderengine::Vector3;

    Vector3 rotate_by_quaternion(Vector3 value, const float quaternion[4])
    {
        const Vector3 q{quaternion[0], quaternion[1], quaternion[2]};
        const Vector3 twiceCross{
            2.0f * (q.y * value.z - q.z * value.y),
            2.0f * (q.z * value.x - q.x * value.z),
            2.0f * (q.x * value.y - q.y * value.x)};
        const Vector3 qCrossTwice{
            q.y * twiceCross.z - q.z * twiceCross.y,
            q.z * twiceCross.x - q.x * twiceCross.z,
            q.x * twiceCross.y - q.y * twiceCross.x};
        return {
            value.x + quaternion[3] * twiceCross.x + qCrossTwice.x,
            value.y + quaternion[3] * twiceCross.y + qCrossTwice.y,
            value.z + quaternion[3] * twiceCross.z + qCrossTwice.z};
    }

    bool normalize(Vector3& value)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (!std::isfinite(lengthSquared) || lengthSquared < 1.0e-12f)
        {
            return false;
        }
        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        value.x *= inverseLength;
        value.y *= inverseLength;
        value.z *= inverseLength;
        return true;
    }

    bool read_vec3(const cgltf_accessor* accessor, cgltf_size index, Vector3& value)
    {
        float components[3]{};
        if (accessor == nullptr || accessor->type != cgltf_type_vec3 ||
            !cgltf_accessor_read_float(accessor, index, components, 3))
        {
            return false;
        }
        value = {components[0], components[1], components[2]};
        return true;
    }
}

namespace softphysics::assets
{
    std::optional<softphysics::renderengine::MeshData> load_glb_mesh(
        const std::filesystem::path& path,
        std::string& errorMessage)
    {
        cgltf_options options{};
        cgltf_data* parsedData = nullptr;
        const std::string pathString = path.string();
        const auto parseResult = cgltf_parse_file(&options, pathString.c_str(), &parsedData);
        if (parseResult != cgltf_result_success)
        {
            if (parsedData != nullptr)
            {
                cgltf_free(parsedData);
            }
            errorMessage = "cgltf could not parse the file (result " +
                           std::to_string(static_cast<int>(parseResult)) + ").";
            return std::nullopt;
        }
        // cgltf owns parsed arrays and loaded buffer data; this guard releases them on every exit.
        const std::unique_ptr<cgltf_data, decltype(&cgltf_free)> data(parsedData, &cgltf_free);

        const auto bufferResult = cgltf_load_buffers(&options, parsedData, pathString.c_str());
        if (bufferResult != cgltf_result_success)
        {
            errorMessage = "cgltf could not load the mesh buffer (result " +
                           std::to_string(static_cast<int>(bufferResult)) + ").";
            return std::nullopt;
        }
        const auto validationResult = cgltf_validate(parsedData);
        if (validationResult != cgltf_result_success)
        {
            errorMessage = "The glTF asset failed validation (result " +
                           std::to_string(static_cast<int>(validationResult)) + ").";
            return std::nullopt;
        }

        const cgltf_node* meshNode = nullptr;
        for (cgltf_size nodeIndex = 0; nodeIndex < parsedData->nodes_count; ++nodeIndex)
        {
            const auto* node = &parsedData->nodes[nodeIndex];
            if (node->mesh != nullptr)
            {
                if (meshNode != nullptr)
                {
                    errorMessage = "This first importer supports one mesh node per GLB.";
                    return std::nullopt;
                }
                meshNode = node;
            }
        }
        if (meshNode == nullptr || meshNode->parent != nullptr)
        {
            errorMessage = "The GLB must contain one root-level mesh node.";
            return std::nullopt;
        }
        if (meshNode->has_matrix)
        {
            errorMessage = "The first importer supports node translation, rotation, and scale, not matrix transforms.";
            return std::nullopt;
        }

        Vector3 nodeScale{1.0f, 1.0f, 1.0f};
        Vector3 nodeTranslation{};
        float nodeRotation[4]{0.0f, 0.0f, 0.0f, 1.0f};
        if (meshNode->has_scale)
        {
            nodeScale = {meshNode->scale[0], meshNode->scale[1], meshNode->scale[2]};
        }
        if (meshNode->has_translation)
        {
            nodeTranslation = {
                meshNode->translation[0], meshNode->translation[1], meshNode->translation[2]};
        }
        if (meshNode->has_rotation)
        {
            for (int component = 0; component < 4; ++component)
            {
                nodeRotation[component] = meshNode->rotation[component];
            }
        }

        if (!std::isfinite(nodeScale.x) || !std::isfinite(nodeScale.y) ||
            !std::isfinite(nodeScale.z) || std::abs(nodeScale.x) < 1.0e-8f ||
            std::abs(nodeScale.y) < 1.0e-8f || std::abs(nodeScale.z) < 1.0e-8f)
        {
            errorMessage = "The mesh node has an invalid scale.";
            return std::nullopt;
        }
        const float rotationLengthSquared = nodeRotation[0] * nodeRotation[0] +
                                            nodeRotation[1] * nodeRotation[1] +
                                            nodeRotation[2] * nodeRotation[2] +
                                            nodeRotation[3] * nodeRotation[3];
        if (!std::isfinite(rotationLengthSquared) || rotationLengthSquared < 1.0e-8f)
        {
            errorMessage = "The mesh node has an invalid rotation.";
            return std::nullopt;
        }
        const float inverseRotationLength = 1.0f / std::sqrt(rotationLengthSquared);
        for (float& component : nodeRotation)
        {
            component *= inverseRotationLength;
        }

        // Flatten supported primitives into one CPU mesh and bake the root node transform.
        softphysics::renderengine::MeshData mesh;
        for (cgltf_size primitiveIndex = 0;
             primitiveIndex < meshNode->mesh->primitives_count;
             ++primitiveIndex)
        {
            const auto& primitive = meshNode->mesh->primitives[primitiveIndex];
            if (primitive.type != cgltf_primitive_type_triangles)
            {
                errorMessage = "The first importer supports triangle-list primitives only.";
                return std::nullopt;
            }

            const auto* positions = cgltf_find_accessor(
                &primitive, cgltf_attribute_type_position, 0);
            const auto* normals = cgltf_find_accessor(
                &primitive, cgltf_attribute_type_normal, 0);
            const auto* colors = cgltf_find_accessor(
                &primitive, cgltf_attribute_type_color, 0);
            if (positions == nullptr || normals == nullptr ||
                positions->count != normals->count || positions->count < 3 ||
                positions->count > std::numeric_limits<std::uint32_t>::max() - mesh.vertices.size())
            {
                errorMessage = "The mesh primitive needs matching position and normal attributes.";
                return std::nullopt;
            }
            if (colors != nullptr && (colors->count != positions->count ||
                (colors->type != cgltf_type_vec3 && colors->type != cgltf_type_vec4)))
            {
                errorMessage = "The mesh color attribute has an unsupported layout.";
                return std::nullopt;
            }

            const auto baseVertex = static_cast<std::uint32_t>(mesh.vertices.size());
            const float baseColor[4] = {
                primitive.material != nullptr ? primitive.material->pbr_metallic_roughness.base_color_factor[0] : 1.0f,
                primitive.material != nullptr ? primitive.material->pbr_metallic_roughness.base_color_factor[1] : 1.0f,
                primitive.material != nullptr ? primitive.material->pbr_metallic_roughness.base_color_factor[2] : 1.0f,
                primitive.material != nullptr ? primitive.material->pbr_metallic_roughness.base_color_factor[3] : 1.0f};

            for (cgltf_size vertexIndex = 0; vertexIndex < positions->count; ++vertexIndex)
            {
                Vector3 position{};
                Vector3 normal{};
                if (!read_vec3(positions, vertexIndex, position) ||
                    !read_vec3(normals, vertexIndex, normal))
                {
                    errorMessage = "Could not read a position or normal from the mesh.";
                    return std::nullopt;
                }

                position = {
                    position.x * nodeScale.x,
                    position.y * nodeScale.y,
                    position.z * nodeScale.z};
                position = rotate_by_quaternion(position, nodeRotation);
                position.x += nodeTranslation.x;
                position.y += nodeTranslation.y;
                position.z += nodeTranslation.z;

                normal = {
                    normal.x / nodeScale.x,
                    normal.y / nodeScale.y,
                    normal.z / nodeScale.z};
                normal = rotate_by_quaternion(normal, nodeRotation);
                if (!normalize(normal))
                {
                    errorMessage = "The mesh contains an invalid normal.";
                    return std::nullopt;
                }

                float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
                if (colors != nullptr)
                {
                    const cgltf_size colorComponents = cgltf_num_components(colors->type);
                    if (!cgltf_accessor_read_float(colors, vertexIndex, color, colorComponents))
                    {
                        errorMessage = "Could not read a vertex color from the mesh.";
                        return std::nullopt;
                    }
                    if (colorComponents == 3)
                    {
                        color[3] = 1.0f;
                    }
                }

                // glTF is right-handed. Reflect Z for this engine's left-handed +Z-forward convention.
                mesh.vertices.push_back({
                    {position.x, position.y, -position.z},
                    {normal.x, normal.y, -normal.z},
                    {color[0] * baseColor[0], color[1] * baseColor[1],
                     color[2] * baseColor[2], color[3] * baseColor[3]}});
            }

            const cgltf_size primitiveIndexCount = primitive.indices != nullptr
                ? primitive.indices->count
                : positions->count;
            if (primitiveIndexCount % 3 != 0 ||
                primitiveIndexCount > std::numeric_limits<std::uint32_t>::max() - mesh.indices.size())
            {
                errorMessage = "The mesh index data does not contain complete triangles.";
                return std::nullopt;
            }

            const cgltf_size firstIndex = mesh.indices.size();
            for (cgltf_size index = 0; index < primitiveIndexCount; ++index)
            {
                const cgltf_size localIndex = primitive.indices != nullptr
                    ? cgltf_accessor_read_index(primitive.indices, index)
                    : index;
                if (localIndex >= positions->count)
                {
                    errorMessage = "The mesh contains an index outside its vertex range.";
                    return std::nullopt;
                }
                mesh.indices.push_back(baseVertex + static_cast<std::uint32_t>(localIndex));
            }

            // Account for the handedness reflection and any mirrored node scale.
            const bool reverseWinding = (-nodeScale.x * nodeScale.y * nodeScale.z) < 0.0f;
            for (cgltf_size index = firstIndex;
                 reverseWinding && index < mesh.indices.size();
                 index += 3)
            {
                std::swap(mesh.indices[index + 1], mesh.indices[index + 2]);
            }
        }

        if (mesh.vertices.empty() || mesh.indices.empty())
        {
            errorMessage = "The mesh node did not contain any triangle geometry.";
            return std::nullopt;
        }
        return mesh;
    }
}
