#include "ParseLeopph3D.hpp"

#include "Compress.hpp"
#include "Deserialize.hpp"
#include "Image.hpp"
#include "Logger.hpp"

#include <bit>
#include <fstream>
#include <memory>
#include <vector>


namespace leopph::convert
{
	namespace
	{
		constexpr u64 HEADER_SZ = 13;
	}


	auto ParseLeopph3D(std::filesystem::path const& path) -> std::optional<Object>
	{
		try
		{
			std::ifstream in{path, std::ios::in | std::ios::binary};

			// disable whitespace skipping
			in.unsetf(std::ios::skipws);

			// failed to open file
			if (!in.is_open())
			{
				internal::Logger::Instance().Error("Can't parse leopph3d file at " + path.string() + " because the file does not exist.");
				return {};
			}

			std::vector<u8> buffer(HEADER_SZ);

			// read header
			in.read(reinterpret_cast<char*>(buffer.data()), HEADER_SZ);

			// failed to read header
			if (in.eof() || in.fail() ||
				buffer[0] != 'x' ||
				buffer[1] != 'd' ||
				buffer[2] != '6' ||
				buffer[3] != '9')
			{
				internal::Logger::Instance().Error("Can't parse leopph3d file at " + path.string() + " because the file is not in valid leopph3d format.");
				return {};
			}

			// parse endianness
			auto const endianness = buffer[4] & 0x80 ? std::endian::little : std::endian::big;

			// parse content size
			auto const contentSize = Deserialize<u64>(std::begin(buffer) + 5, endianness);

			// get the size of the compressed contents
			in.seekg(0, std::ios_base::end);
			auto const comprSz = static_cast<u64>(in.tellg()) - HEADER_SZ;

			// read rest of the file
			buffer.resize(comprSz);
			in.seekg(HEADER_SZ, std::ios_base::beg);
			in.read(reinterpret_cast<char*>(buffer.data()), comprSz);

			std::vector<u8> uncompressed;

			// uncompress data
			if (compress::Uncompress({std::begin(buffer), std::end(buffer)}, contentSize, uncompressed) != compress::Error::None)
			{
				internal::Logger::Instance().Error("Couldn't parse leopph3d file at " + path.string() + " because the contents failed to uncompress.");
				return {};
			}

			auto it = std::cbegin(uncompressed);

			// number of images
			auto const numImgs = Deserialize<u64>(it, endianness);

			// all images
			std::vector<Image> imgs;
			imgs.reserve(numImgs);

			// parse image data
			for (u64 i = 0; i < numImgs; i++)
			{
				imgs.push_back(DeserializeImage(it, endianness));
			}

			// number of materials
			auto const numMats = Deserialize<u64>(it, endianness);

			// all materials
			std::vector<Material> mats;
			mats.reserve(numMats);

			// parse material data
			for (u64 i = 0; i < numMats; i++)
			{
				mats.push_back(DeserializeMaterial(it, endianness));
			}

			// number of meshes
			auto const numMeshes = Deserialize<u64>(it, endianness);

			// all meshes
			std::vector<Mesh> meshes;
			meshes.reserve(numMeshes);

			// parse mesh data
			for (u64 i = 0; i < numMeshes; i++)
			{
				meshes.push_back(DeserializeMesh(it, endianness));
			}

			return Object
			{
				.Textures = std::move(imgs),
				.Materials = std::move(mats),
				.Meshes = std::move(meshes)
			};
		}
		catch (...)
		{
			internal::Logger::Instance().Error("Couldn't parse leopph3d file at " + path.string() + ". The file may be corrupt.");
			return {};
		}
	}
}