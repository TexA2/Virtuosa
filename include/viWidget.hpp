#ifndef VI_WIDGET
#define VI_WIDGET

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>


#include "string"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "nfd.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <viCamera.hpp>
#include <viShader.hpp>
#include <viData.hpp>

namespace viWidget {


// ============================================================================
// НАСТРОЙКИ И КОНФИГУРАЦИЯ
// ===========================================================================
    struct WindowSettings {
        int width = 1280;
        int height = 1024;
        std::string title = "Virtuosa Point Cloud Viewer";
        bool fullscreen = false;
        bool vsync = false;
        int msaaSamples = 4;
    };

    struct RenderSettings {
            struct ClearColor {
                float r = 0.25f;
                float g = 0.4f;
                float b = 0.48f;
                float a = 1.0f;
            } clearColor;
            
            struct PointCloud {
                struct ColorMode {
                    bool useIntensity = false;
                    glm::vec4 uniformColor = {1.0f, 1.0f, 0.0f, 1.0f};
                } colorMode;
                
                float pointSize = 2.0f;
                bool showBoundingBox = false;
            } pointCloud;
            
            struct Camera {
                float fov = 75.0f;
                float nearPlane = 0.1f;
                float farPlane = 1000.0f;
                bool usePerspective = true;
            } camera;
    };


// Вот эти глобальные переменные пойдут в другую струтуру типо MainBar
    extern bool show_BackroundColor;
    extern bool show_pointColor;
    extern bool buttonQuit;


// ============================================================================
// ОПИСАНИЕ КЛАССА
// ===========================================================================

    class viMainWidget {
        public:

            viMainWidget(const viMainWidget&) = delete;
            viMainWidget& operator=(const viMainWidget&) = delete;
            
            viMainWidget(const WindowSettings& windowSettings = {}) : _windowSettings(windowSettings) {
                viewCamera = nullptr;
                window = nullptr;
                cloudShader = nullptr;
                cloudData= nullptr;
            };

            ~viMainWidget() = default;

            //void initialize(); // запихаю сюда все функциии c именем init

            GLFWwindow* initMainWindow();
            void initCamera();
            void initGui();
            void initShader();
            void initCloudData();

            static void resizeWindow(GLFWwindow* window, int width, int heigth);

            void ShowExampleAppMainMenuBar(bool& projType);  //в UI
            void ShowExampleMenuFile();                      // в UI

            std::shared_ptr<viCamera::Camera> getCamera() const;
            std::shared_ptr<viShader::Shader> getShader() const;
            std::shared_ptr<viData::viManageData> getCloudData();

        private:
            std::shared_ptr<GLFWwindow> window;
            std::shared_ptr<viCamera::Camera> viewCamera;
            std::shared_ptr<viShader::Shader> cloudShader;
            std::shared_ptr<viData::viManageData> cloudData;
            

            WindowSettings _windowSettings;
    };
}

#endif