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

namespace viWidget {

// Вот эти глобальные переменные пойдут в другую струтуру типо MainBar
    extern bool show_BackroundColor;
    extern bool show_pointColor;
    extern bool buttonQuit;

    struct CloudData {
        pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud;
        std::vector<float> intensity;
        GLuint VAO, instanceVBO;
        GLuint intensityVBO;
        float cloudMinX = std::numeric_limits<float>::max(); 
        float cloudMinY = std::numeric_limits<float>::max(); 
        float cloudMinZ = std::numeric_limits<float>::max();
        float cloudMaxX = std::numeric_limits<float>::lowest();
        float cloudMaxY = std::numeric_limits<float>::lowest();
        float cloudMaxZ = std::numeric_limits<float>::lowest();
        float cloudIntensityMin = std::numeric_limits<float>::max();
        float cloudIntensityMax = std::numeric_limits<float>::lowest();
        bool cloudOpen = false;
    };


    class viMainWidget {
        public:
            
            viMainWidget(uint width, uint height): _width(width), _height(height) {
                viewCamera = nullptr;
                window = nullptr;
            };

            ~viMainWidget()
            {
                if (viewCamera)
                    delete viewCamera;

                if (window)
                    glfwDestroyWindow(window);
            };

            GLFWwindow* initMainWindow();
            void initCamera();
            void initGui();

            static void resizeWindow(GLFWwindow* window, int width, int heigth);

            void viPointcloudOpen(std::string path);
            void cloudBuffer();

            void ShowExampleAppMainMenuBar(bool& projType);
            void ShowExampleMenuFile();

            void calculateCloudBounds();

            viCamera::Camera* getCamera();

            //пока одиночная переменная, потом надо будет сделать, что то типо массива таких
            // перемнных чтоб можно было много облаков одновременно открывать
            CloudData viCloud;
        private:
            GLFWwindow* window;
            viCamera::Camera* viewCamera;

            uint _width;
            uint _height;
    };


    void intensityToColor(float intensity, float& r, float& g, float& b);
}

#endif