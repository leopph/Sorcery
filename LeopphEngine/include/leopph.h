#pragma once

/*------------------------------------------
Standard LeopphEngine API
See individual headers for more information.
------------------------------------------*/

#include "../src/components/Component.hpp"
#include "../src/components/Behavior.hpp"
#include "../src/components/Camera.hpp"
#include "../src/components/lighting/AmbientLight.hpp"
#include "../src/components/lighting/DirLight.hpp"
#include "../src/components/lighting/PointLight.hpp"
#include "../src/components/models/Model.hpp"
#include "../src/config/settings.h"
#include "../src/hierarchy/Object.hpp"
#include "../src/input/input.h"
#include "../src/math/leopphmath.h"
#include "../src/math/matrix.h"
#include "../src/math/quaternion.h"
#include "../src/math/vector.h"
#include "../src/misc/camerabackground.h"
#include "../src/misc/color.h"
#include "../src/misc/skybox.h"
#include "../src/timing/leopphtime.h"
#include "../src/util/ObjectProperties.hpp"

/*-----------------------------------------------------
Define LEOPPH_ENTRY in exactly one of your source files
to create an entry point in your application.
-----------------------------------------------------*/

#ifdef LEOPPH_ENTRY
#include "../src/entry/entry.h"
#endif