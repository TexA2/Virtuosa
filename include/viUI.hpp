#ifndef VI_UI
#define VI_UI

#include <viCamera.hpp>
#include <viData.hpp>
#include <cstdio>
#include <memory>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "nfd.hpp"


namespace viWidget
{
    struct WindowSettings;
} 


namespace viUI {

    class viManageUI {
        public:
            viManageUI(std::shared_ptr<viData::viManageData> &cloudData,
                       std::shared_ptr<viCamera::Camera> &viewCamera,
                       viWidget::WindowSettings &windowsSetting) :
                        _cloudData(cloudData),
                        _viewCamera(viewCamera),
                        _windowsSetting(windowsSetting)
                        {
                            clear_color = glm::vec4(20.0f / 255.0f,
                                                    13.0f / 255.0f,
                                                    24.0f / 255.0f,
                                                    1.00f);
                        }

            ~viManageUI() = default;


            void ShowExampleAppMainMenuBar();
            void ShowExampleMenuFile();
            void showObjectPanel();
            void renderUI(GLFWwindow* window);


            bool show_BackroundColor = false;
            bool show_pointColor = false;
            bool buttonQuit = false;

            glm::vec4 clear_color; // потом в рендер
        private:
            std::weak_ptr<viData::viManageData> _cloudData;
            std::weak_ptr<viCamera::Camera> _viewCamera;
            viWidget::WindowSettings& _windowsSetting;

             std::string selectedCloudId;
    };
}

#endif