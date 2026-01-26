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


namespace viUI {

    class viManageUI {
        public:
            viManageUI(std::shared_ptr<viData::viManageData> &cloudData,
                        std::shared_ptr<viCamera::Camera> &viewCamera) :
                        _cloudData(cloudData),
                        _viewCamera(viewCamera)
                        {
                            clear_color = ImVec4(0.25f, 0.4f, 0.48f, 1.00f);
                            point_color = ImVec4(1.f, 1.f, 0.f, 1.00f);
                        }

            ~viManageUI() = default;


            void ShowExampleAppMainMenuBar();
            void ShowExampleMenuFile();
            void renderUI(GLFWwindow* window);


            bool show_BackroundColor = false;
            bool show_pointColor = false;
            bool buttonQuit = false;
            bool show_intensity_color = false;

            ImVec4 clear_color; // потом в рендер
            ImVec4 point_color; // потом в рендер
        
        private:
            std::weak_ptr<viData::viManageData> _cloudData;
            std::weak_ptr<viCamera::Camera> _viewCamera;
    };
}

#endif