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

// Пока сюда засуну пикер шейдер , потом буду шейдер переписывать
        // Создание буфера для результатов
        resultData = {-1, 3.40282e+38f, 0};
        glGenBuffers(1, &resultBuffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, resultBuffer);

        glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(ResultData), &resultData, GL_DYNAMIC_READ);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, resultBuffer);
        //конец

        computeShader = glCreateShader(GL_COMPUTE_SHADER);

        std::ifstream fileStream("../shader/pointPicker.comp", std::ios::in);
        std::stringstream ShaderStream;
        ShaderStream << fileStream.rdbuf();
        fileStream.close();
        std::string Code = ShaderStream.str();
     
        const char* shaderSource = Code.c_str();
        glShaderSource(computeShader, 1, &shaderSource, NULL);
        glCompileShader(computeShader);


        GLint success;
        glGetShaderiv(computeShader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(computeShader, 512, NULL, infoLog);
            printf("Ошибка компиляции compute шейдера: %s\n", infoLog);
        }

        // Создание программы
        computeProgram = glCreateProgram();
        glAttachShader(computeProgram, computeShader);
        glLinkProgram(computeProgram);

        // Проверка на ошибки линковки
        glGetProgramiv(computeProgram, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(computeProgram, 512, NULL, infoLog);
            printf("Ошибка линковки программы: %s\n", infoLog);
        }

        glDeleteShader(computeShader);

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


            bool show_BackroundColor = false;
            bool show_pointColor = false;
            bool buttonQuit = false;

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
            GLuint computeShader;
            GLuint computeProgram;
    };
}

#endif