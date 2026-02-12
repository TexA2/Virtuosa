#ifndef VI_WIDGET
#define VI_WIDGET

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <viCamera.hpp>
#include <viShader.hpp>
#include <viData.hpp>
#include <viUI.hpp>
#include <optional>

#include "string"

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
        float objectPanelWidth = 0.12f;
    };

// ============================================================================
// ОПИСАНИЕ КЛАССА
// ===========================================================================

    class viMainWidget {
        public:

            viMainWidget(const viMainWidget&) = delete;
            viMainWidget& operator=(const viMainWidget&) = delete;
            
            viMainWidget(const WindowSettings& windowSettings = {}) : _windowSettings(windowSettings) {
                if (initMainWindow())
                {
                    initGui();
                    initCamera();
                    initShader();
                    initCloudData();
                    initUI();
                } else {
                    std::cout << "Error initMainWindow" << std::endl;
                }
            };

            ~viMainWidget() {
                ImGui_ImplOpenGL3_Shutdown();
                ImGui_ImplGlfw_Shutdown();
                ImGui::DestroyContext();

                glfwTerminate();
            };

            std::optional<bool>  initMainWindow();
            void initCamera();
            void initGui();
            void initShader();
            void initCloudData();
            void initUI();

            float getObjectPanelWidth() { return _windowSettings.objectPanelWidth; }

            static void resizeWindow(GLFWwindow* window, int width, int heigth);

            void render();

            std::shared_ptr<viCamera::Camera> getCamera() const;
            std::shared_ptr<viShader::Shader> getShader() const;
            std::shared_ptr<viData::viManageData> getCloudData();
            std::shared_ptr<viUI::viManageUI> getMenu();
            GLFWwindow* getWindow() { return window.get();};

        private:
            std::shared_ptr<GLFWwindow> window;
            std::shared_ptr<viCamera::Camera> viewCamera;
            std::shared_ptr<viShader::Shader> cloudShader;
            std::shared_ptr<viData::viManageData> cloudData;
            std::shared_ptr<viUI::viManageUI> menuUI;
            

            WindowSettings _windowSettings;
    };
}

#endif