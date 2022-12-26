#pragma once

#include "Core.hpp"
#include "Guid.hpp"
#include "YamlInclude.hpp"

#include <concepts>
#include <set>
#include <span>
#include <string>

namespace leopph {
	class Object {
		struct GuidObjectLess {
			using is_transparent = void;
			[[nodiscard]] auto operator()(Guid const& left, Guid const& right) const -> bool;
			[[nodiscard]] auto operator()(Object const* left, Guid const& right) const -> bool;
			[[nodiscard]] auto operator()(Guid const& left, Object const* right) const -> bool;
			[[nodiscard]] auto operator()(Object const* left, Object const* right) const -> bool;
		};

		LEOPPHAPI static std::set<Object*, GuidObjectLess> sAllObjects;

		Guid mGuid{ Guid::Generate() };
		std::string mName;

	public:
		enum class Type {
			Entity,
			Transform,
			Camera,
			Behavior,
			CubeModel,
			DirectionalLight,
			Material,
			Mesh
		};

		[[nodiscard]] LEOPPHAPI static auto FindObjectByGuid(Guid const& guid) -> Object*;

		template<std::derived_from<Object> T>
		[[nodiscard]] static auto FindObjectOfType() -> T*;

		template<std::derived_from<Object> T>
		static auto FindObjectsOfType(std::vector<T*>& out) -> std::vector<T*>&;

		template<std::derived_from<Object> T>
		[[nodiscard]] auto FindObjectsOfType() -> std::vector<T*>;

		template<std::derived_from<Object> T>
		static auto FindObjectsOfType(std::vector<Object*>& out) -> std::vector<Object*>&;

		LEOPPHAPI Object();
		Object(Object const& other) = delete;
		Object(Object&& other) = delete;

		LEOPPHAPI virtual ~Object();

		auto operator=(Object const& other) -> void = delete;
		auto operator=(Object&& other) -> void = delete;

		[[nodiscard]] LEOPPHAPI auto GetGuid() const -> Guid const&;
		LEOPPHAPI auto SetGuid(Guid const& guid) -> void;

		[[nodiscard]] LEOPPHAPI auto GetName() const noexcept -> std::string_view;
		LEOPPHAPI auto SetName(std::string name) noexcept -> void;

		[[nodiscard]] virtual auto GetSerializationType() const -> Type = 0;

		LEOPPHAPI virtual auto SerializeTextual(YAML::Node& node) const -> void;
		LEOPPHAPI virtual auto DeserializeTextual(YAML::Node const& node) -> void;

		LEOPPHAPI virtual auto SerializeBinary(std::vector<u8>& out) const -> void;

		struct BinaryDeserializationResult {
			u64 numBytesConsumed;
		};

		LEOPPHAPI [[nodiscard]] virtual auto DeserializeBinary(std::span<u8 const> bytes) -> BinaryDeserializationResult;
	};


	template<std::derived_from<Object> T>
	auto Object::FindObjectOfType() -> T* {
		for (auto const obj : sAllObjects) {
			if (auto const castObj{ dynamic_cast<T*>(obj) }; castObj) {
				return castObj;
			}
		}
		return nullptr;
	}


	template<std::derived_from<Object> T>
	auto Object::FindObjectsOfType(std::vector<T*>& out) -> std::vector<T*>& {
		out.clear();
		for (auto const obj : sAllObjects) {
			if (auto const castObj{ dynamic_cast<T*>(obj) }; castObj) {
				out.emplace_back(castObj);
			}
		}
		return out;
	}


	template<std::derived_from<Object> T>
	auto Object::FindObjectsOfType() -> std::vector<T*> {
		std::vector<T*> ret;
		FindObjectsOfType<T>(ret);
		return ret;
	}


	template<std::derived_from<Object> T>
	auto Object::FindObjectsOfType(std::vector<Object*>& out) -> std::vector<Object*>& {
		out.clear();
		for (auto const obj : sAllObjects) {
			if (dynamic_cast<T*>(obj)) {
				out.emplace_back(obj);
			}
		}
		return out;
	}
}
