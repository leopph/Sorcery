#include "PropertiesWindow.hpp"


namespace sorcery::mage {
PropertiesWindow::PropertiesWindow(Application& app) :
  mApp{&app} { }


auto PropertiesWindow::Draw() -> void {
  ImGui::SetNextWindowSizeConstraints(ImVec2{300, 200}, ImVec2{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()});

  if (ImGui::Begin("Object Properties")) {
    if (auto const selectedObj{mApp->GetSelectedObject()}) {
      bool changed{false};
      selectedObj->OnDrawProperties(changed);

      if (changed) {
        if (auto const nativeRes{rttr::rttr_cast<ObserverPtr<NativeResource>>(selectedObj)}; nativeRes && mApp->GetResourceDatabase().IsSavedResource(*nativeRes)) {
          mApp->GetResourceDatabase().SaveResource(*nativeRes);
        }
      }
    }
  }
  ImGui::End();
}
}
