#ifndef VI_WIDGET
#define VI_WIDGET

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "string"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "nfd.hpp"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <glm/glm.hpp>

namespace viWidget {

// Вот эти глобальные переменные пойдут в другую струтуру типо MainBar
    extern bool show_BackroundColor;
    extern bool show_pointColor;
    extern bool buttonQuit;

    class viMainWidget {
        public:
            
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

            viMainWidget(): _width(1280), _height(1024) {};
            ~viMainWidget() = default;

            GLFWwindow* initMainWindow();
            void initGui();

            static void resizeWindow(GLFWwindow* window, int width, int heigth);

            void viPointcloudOpen(std::string path);
            void cloudBuffer();

            void ShowExampleAppMainMenuBar(bool& projType);
            void ShowExampleMenuFile();

            void calculateCloudBounds();

            //пока одиночная переменная, потом надо будет сделать, что то типо массива таких
            // перемнных чтоб можно было много облаков одновременно открывать
            CloudData viCloud;
        private:
            GLFWwindow* window;

            uint _width;
            uint _height;

    };


    void intensityToColor(float intensity, float& r, float& g, float& b);
}

#endif