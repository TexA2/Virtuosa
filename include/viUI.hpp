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

#include <fstream>
#include <sstream>


namespace viWidget
{
    struct WindowSettings;
} 

enum class Mode : uint8_t { 
    viewMode,
    selectMode,
    transformMode,
    drawMode
};

    struct ResultData {
        int selectedIndex;
        float minDistance;
        uint foundPoint;
        float debugCounter;
    };


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

                            curMode = Mode::viewMode;
                        }

            ~viManageUI() = default;


            void ShowExampleAppMainMenuBar();
            void ShowExampleMenuFile();
            void showObjectPanel();
            void renderUI(GLFWwindow* window);
            void ModeButton(const char* label, Mode buttonMode, Mode& curMode, const ImVec2& size);
            void viewMode(GLFWwindow* window);
            void selectMode(GLFWwindow* window);
            void drawMode(GLFWwindow* window);
            void transformMode(GLFWwindow* window);

            glm::vec4 clear_color;
        private:
            std::weak_ptr<viData::viManageData> _cloudData;
            std::weak_ptr<viCamera::Camera> _viewCamera;
            viWidget::WindowSettings& _windowsSetting;

            std::string selectedCloudId;

            Mode curMode;


            // для теста
            ResultData resultData;
            GLuint resultBuffer;
           // GLuint computeShader;
           // GLuint computeProgram;

            bool show_BackroundColor = false;
            bool showTransform_ = false;
            bool show_pointColor = false;
            bool buttonQuit_ = false;
    };
}

#endif