#ifndef VI_GUI
#define VI_GUI


#include "string"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "nfd.hpp"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <glm/glm.hpp>

#include "globals.h"

namespace viGui {

    extern bool show_BackroundColor;
    extern bool show_pointColor;
    extern bool cloudOpen;
    extern int cloud_size;

    void ShowExampleMenuFile(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity);
    void ShowExampleAppMainMenuBar(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity);
    
    void pointCloudOpen(std::string path, std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity);
    void intensityToColor(float intensity, float& r, float& g, float& b);
}

#endif