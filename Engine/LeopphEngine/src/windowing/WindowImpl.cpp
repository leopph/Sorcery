#include "WindowImpl.hpp"

#include "GLWindow.hpp"
#include "../config/Settings.hpp"
#include "Logger.hpp"

#include <stdexcept>


namespace leopph::internal
{
	auto WindowImpl::Create() -> std::unique_ptr<WindowImpl>
	{
		std::unique_ptr<WindowImpl> ret;

		switch (Settings::Instance().GetGraphicsApi())
		{
			case Settings::GraphicsApi::OpenGl:
				 ret = std::make_unique<GlWindow>();
		}

		if (!ret)
		{
			const auto errMsg{"Failed to create window: the selected graphics API is not supported."};
			Logger::Instance().Critical(errMsg);
			throw std::runtime_error{errMsg};
		}

		return ret;
	}
}