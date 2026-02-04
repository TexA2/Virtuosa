#ifndef VI_DATA
#define VI_DATA


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <memory>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <algorithm>


namespace viData {

    static constexpr float MAX_VAL = std::numeric_limits<float>::max();
    static constexpr float MIN_VAL = std::numeric_limits<float>::lowest();

    struct CloudData {

        pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud = nullptr;
        std::vector<float> intensity;                                 // intensity color for point
        glm::vec4 point_color = glm::vec4(1.f, 1.f, 0.f, 1.0f);

        //создать струкру настройки и запихать туда данные
        bool isSelected = false;
        bool isVisible = true;


        struct Buffer
        {
            GLuint VAO          = 0;
            GLuint pointVBO     = 0;
            GLuint intensityVBO = 0;
        } buffer;
        
        struct Bounds
        {
            glm::vec3 min = glm::vec3(MAX_VAL);                       // x , y , z
            glm::vec3 max = glm::vec3(MIN_VAL);                       // x , y , z
            glm::vec2 cloudIntensity = glm::vec2(MAX_VAL, MIN_VAL);   // min , max
        } bounds;
    };

    class viManageData {

        public:

            viManageData() {};
            ~viManageData() = default;

            void pointCloudOpen(std::string path);
            void cloudBuffer(std::shared_ptr<CloudData> cloud);
            void calculateCloudBounds(std::shared_ptr<CloudData> cloud);

            std::unordered_map<std::string, std::shared_ptr<CloudData>> cloudCache;

        private:
            void intensityToColor(float intensity, float& r, float& g, float& b);
            std::string getFileName(const std::string& path);
    };

}

#endif