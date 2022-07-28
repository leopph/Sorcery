#include "DecodeGeneric.hpp"

#include "Image.hpp"
#include "Logger.hpp"
#include "Matrix.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <iostream>
#include <optional>
#include <queue>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>


namespace leopph::convert
{
	namespace
	{
		// Maps texture types to an encoding to be used during interpretation
		std::unordered_map<aiTextureType, ColorEncoding> const gTexTypeToEncoding
		{
			{aiTextureType_DIFFUSE, ColorEncoding::SRGB},
			{aiTextureType_SPECULAR, ColorEncoding::Linear},
			{aiTextureType_OPACITY, ColorEncoding::SRGB}
		};


		// Transposes the matrix
		Matrix4 convert(aiMatrix4x4 const& aiMat)
		{
			return Matrix4
			{
				aiMat.a1, aiMat.b1, aiMat.c1, aiMat.d1,
				aiMat.a2, aiMat.b2, aiMat.c2, aiMat.d2,
				aiMat.a3, aiMat.b3, aiMat.c3, aiMat.d3,
				aiMat.a4, aiMat.b4, aiMat.c4, aiMat.d4
			};
		}



		Vector3 convert(aiVector3D const& aiVec)
		{
			return Vector3{aiVec.x, aiVec.y, aiVec.z};
		}



		Color convert(aiColor3D const& aiCol)
		{
			return Color{static_cast<unsigned char>(aiCol.r * 255), static_cast<unsigned char>(aiCol.g * 255), static_cast<unsigned char>(aiCol.b * 255)};
		}



		// Returns the index of the texture image corresponding to the target texture type in the passed material.
		// If the texture is not yet loaded, it gets loaded and stored in the vector along with its source path and index in the map.
		// Returns an empty optional if the texture could not be found or loaded.
		std::optional<std::size_t> get_texture(aiMaterial const* const mat, aiTextureType const texType, std::vector<Image>& textures, std::unordered_map<std::string, std::size_t>& idToInd, std::filesystem::path const& rootPath)
		{
			if (aiString texPath; mat->GetTexture(texType, 0, &texPath) == aiReturn_SUCCESS)
			{
				if (idToInd.contains(texPath.C_Str()))
				{
					return idToInd[texPath.C_Str()];
				}

				textures.emplace_back(rootPath / texPath.C_Str(), gTexTypeToEncoding.at(texType), ImageOrientation::FlipVertical);
				return idToInd[texPath.C_Str()] = textures.size() - 1;
			}

			return {};
		}



		std::pair<std::vector<Material>, std::vector<Image>> process_materials(std::span<aiMaterial* const> const aiMats, std::filesystem::path const& rootPath)
		{
			std::unordered_map<std::string, std::size_t> idToInd;
			std::vector<Image> textures;
			std::vector<Material> materials;

			for (auto const* const aiMat : aiMats)
			{
				Material mat;

				if (f32 opacity; aiMat->Get(AI_MATKEY_OPACITY, opacity) == aiReturn_SUCCESS)
				{
					mat.opacity = opacity;
				}

				if (aiColor3D diffClr; aiMat->Get(AI_MATKEY_COLOR_DIFFUSE, diffClr) == aiReturn_SUCCESS)
				{
					mat.diffuseColor = convert(diffClr);
				}

				if (aiColor3D specClr; aiMat->Get(AI_MATKEY_COLOR_SPECULAR, specClr) == aiReturn_SUCCESS)
				{
					mat.specularColor = convert(specClr);
				}

				if (ai_real gloss; aiMat->Get(AI_MATKEY_SHININESS, gloss) == aiReturn_SUCCESS)
				{
					mat.gloss = gloss;
				}

				if (int twoSided; aiMat->Get(AI_MATKEY_TWOSIDED, twoSided) == aiReturn_SUCCESS)
				{
					mat.twoSided = !twoSided;
				}

				mat.diffuseMap = get_texture(aiMat, aiTextureType_DIFFUSE, textures, idToInd, rootPath);
				mat.specularMap = get_texture(aiMat, aiTextureType_SPECULAR, textures, idToInd, rootPath);
				mat.opacityMap = get_texture(aiMat, aiTextureType_OPACITY, textures, idToInd, rootPath);

				// If the diffuse map has an alpha channel, and we couldn't parse an opacity map
				// We assume that the transparency comes from the diffuse alpha, so we steal it
				// And create an opacity map from that.
				if (!mat.opacityMap.has_value() && mat.diffuseMap.has_value() && textures[mat.diffuseMap.value()].Channels() == 4)
				{
					auto& tex = textures[mat.diffuseMap.value()];
					textures.push_back(tex.ExtractChannel(3));
					mat.opacityMap = textures.size() - 1;
				}

				materials.push_back(mat);
			}

			return {std::move(materials), std::move(textures)};
		}



		std::vector<Vertex> process_vertices(aiMesh const* mesh, Matrix4 const& trafo)
		{
			std::vector<Vertex> vertices;

			for (unsigned i = 0; i < mesh->mNumVertices; i++)
			{
				Vertex vertex
				{
					.position = Vector3{Vector4{convert(mesh->mVertices[i]), 1} * trafo},
					.normal = Vector3{Vector4{convert(mesh->mNormals[i]), 0} * trafo},
					.uv = [mesh, i]
					{
						for (std::size_t j = 0; j < AI_MAX_NUMBER_OF_TEXTURECOORDS; j++)
						{
							if (mesh->HasTextureCoords(static_cast<unsigned>(j)))
							{
								return Vector2{convert(mesh->mTextureCoords[j][i])};
							}
						}
						return Vector2{};
					}()
				};

				vertices.push_back(vertex);
			}

			return vertices;
		}



		std::vector<unsigned> process_indices(aiMesh const* mesh)
		{
			std::vector<unsigned> indices;

			for (unsigned i = 0; i < mesh->mNumFaces; i++)
			{
				for (unsigned j = 0; j < mesh->mFaces[i].mNumIndices; j++)
				{
					indices.push_back(mesh->mFaces[i].mIndices[j]);
				}
			}

			return indices;
		}



		void log_primitive_error(aiMesh const* const mesh, std::filesystem::path const& path)
		{
			std::string msg{"Ignoring mesh without triangles in model at path ["};
			msg += path.string();
			msg += "]. Primitives are";

			if (mesh->mPrimitiveTypes & aiPrimitiveType_POINT)
			{
				msg += " [points]";
			}

			if (mesh->mPrimitiveTypes & aiPrimitiveType_LINE)
			{
				msg += " [lines]";
			}

			if (mesh->mPrimitiveTypes & aiPrimitiveType_POLYGON)
			{
				msg += " [N>3 polygons]";
			}

			msg += ".";
			internal::Logger::Instance().Debug(msg);
		}
	}



	std::optional<Object> decode_generic_3d_asset(std::filesystem::path const& path)
	{
		Assimp::Importer importer;
		auto const* scene = importer.ReadFile(path.string(),
		                                      aiProcess_JoinIdenticalVertices |
		                                      aiProcess_Triangulate |
		                                      aiProcess_SortByPType |
		                                      aiProcess_GenUVCoords |
		                                      aiProcess_GenNormals);

		if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
		{
			std::cerr << "Error parsing model from file at " << path << ": " << importer.GetErrorString() << '\n';
			return {};
		}

		Object object;

		auto [materials, textures] = process_materials(std::span{scene->mMaterials, scene->mNumMaterials}, path.parent_path());

		object.materials = std::move(materials);
		object.textures = std::move(textures);

		std::queue<std::pair<aiNode const*, Matrix4>> queue;
		queue.emplace(scene->mRootNode, convert(scene->mRootNode->mTransformation) * Matrix4{1, 1, -1, 1});

		while (!queue.empty())
		{
			auto& [node, trafo] = queue.front();

			for (std::size_t i = 0; i < node->mNumMeshes; ++i)
			{
				// aiProcess_SortByPType will separate mixed-primitive meshes, so every mesh in theory should be clean and only contain one kind of primitive.
				// Testing for one type only is therefore safe, but triangle meshes have to be checked for NGON encoding too.
				if (auto const* const mesh = scene->mMeshes[node->mMeshes[i]]; mesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE)
				{
					if (mesh->mPrimitiveTypes & aiPrimitiveType_NGONEncodingFlag)
					{
						internal::Logger::Instance().Debug("Found NGON encoded mesh in model at path [" + path.string() + "].");
						// TODO currently ignoring NGON property
					}

					object.meshes.emplace_back(process_vertices(mesh, trafo), process_indices(mesh), mesh->mMaterialIndex);
				}
				else
				{
					log_primitive_error(mesh, path);
				}
			}

			for (std::size_t i = 0; i < node->mNumChildren; ++i)
			{
				queue.emplace(node->mChildren[i], convert(node->mChildren[i]->mTransformation) * trafo);
			}

			queue.pop();
		}

		return object;
	}
}
