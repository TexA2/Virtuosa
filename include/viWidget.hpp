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

extern bool initCloud;

namespace viWidget {

    class viMainWidget {
        public:
            viMainWidget(): _width(1280), _height(1024) {};
            ~viMainWidget() = default;

            GLFWwindow* initMainWindow();
            void initGui();

            static void resizeWindow(GLFWwindow* window, int width, int heigth);


        private:
            GLFWwindow* window;

            uint _width;
            uint _height;

    };

    

    extern bool show_BackroundColor;
    extern bool show_pointColor;
    extern bool cloudOpen;
    extern int cloud_size;

    extern bool buttonQuit;


    void ShowExampleMenuFile(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity);
    void ShowExampleAppMainMenuBar(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity);
    
    void pointCloudOpen(std::string path, std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity);
    void intensityToColor(float intensity, float& r, float& g, float& b);
}

#endif