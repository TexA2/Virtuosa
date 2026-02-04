#include <viData.hpp>

namespace viData {

    std::string viManageData::getFileName(const std::string& path) {
        std::filesystem::path obj(path);
        return obj.filename().string();
    }

    void viManageData::pointCloudOpen(std::string path) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud);
        std::string name = getFileName(path);

        if (cloudCache.find(name) == cloudCache.end())
        {
            std::shared_ptr<CloudData> temp_cloud = std::make_shared<CloudData> ();
            temp_cloud->_cloud = cloud;

            calculateCloudBounds(temp_cloud);

            for (uint i = 0; i < temp_cloud->_cloud->size(); ++i)
            {
                float normalized_i = (temp_cloud->_cloud->points[i].intensity - temp_cloud->bounds.cloudIntensity.x)
                                    / (temp_cloud->bounds.cloudIntensity.y- temp_cloud->bounds.cloudIntensity.x);  

                float r, g, b;
                intensityToColor(normalized_i, r, g, b);

                temp_cloud->intensity.push_back(r);
                temp_cloud->intensity.push_back(g);
                temp_cloud->intensity.push_back(b);
            }
            cloudBuffer(temp_cloud);

            cloudCache[name] = temp_cloud;
        } else 
        {
            // TODO :: Делать имя: имя_copy
            std::cout << "Object ush dobavlen " << std::endl;
        }
    }


    void viManageData::calculateCloudBounds(std::shared_ptr<CloudData> cloud) {
        
        for (const auto& p : cloud->_cloud->points) 
        {
            cloud->bounds.min.x = std::min(cloud->bounds.min.x, p.x);
            cloud->bounds.min.y = std::min(cloud->bounds.min.y, p.y);
            cloud->bounds.min.z = std::min(cloud->bounds.min.z, p.z);

            cloud->bounds.max.x = std::max(cloud->bounds.max.x, p.x);
            cloud->bounds.max.y = std::max(cloud->bounds.max.y, p.y);
            cloud->bounds.max.z = std::max(cloud->bounds.max.z, p.z);

            cloud->bounds.cloudIntensity.x = std::min(cloud->bounds.cloudIntensity.x, p.intensity);
            cloud->bounds.cloudIntensity.y = std::max(cloud->bounds.cloudIntensity.y, p.intensity);
        }
    }

    void viManageData::cloudBuffer(std::shared_ptr<CloudData> cloud) {

        glGenVertexArrays(1, &cloud->buffer.VAO);
        glGenBuffers(1, &cloud->buffer.pointVBO); 
        glGenBuffers(1, &cloud->buffer.intensityVBO);

        glBindVertexArray(cloud->buffer.VAO);

        glBindBuffer(GL_ARRAY_BUFFER, cloud->buffer.pointVBO);
        glBufferData(GL_ARRAY_BUFFER, cloud->_cloud->size() * sizeof(pcl::PointXYZI), cloud->_cloud->data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZI), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribDivisor(0, 1); 

        glBindBuffer(GL_ARRAY_BUFFER, cloud->buffer.intensityVBO);
        glBufferData(GL_ARRAY_BUFFER, cloud->intensity.size() * sizeof(float), cloud->intensity.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);

        glVertexAttribDivisor(1, 1);


        //отвязка параметров чтоб случайно не изменить   
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }



    void viManageData::intensityToColor(float intensity, float& r, float& g, float& b) {
        intensity = std::max(0.0f, std::min(1.0f, intensity));
        
        float r4 = 4.0f * intensity;
        
        float rf = std::min(r4 - 1.5f, -r4 + 4.5f);
        float gf = std::min(r4 - 0.5f, -r4 + 3.5f);
        float bf = std::min(r4 + 0.5f, -r4 + 2.5f);
        
        rf = std::max(0.0f, std::min(1.0f, rf));
        gf = std::max(0.0f, std::min(1.0f, gf));
        bf = std::max(0.0f, std::min(1.0f, bf));
        
        r = static_cast<uint8_t>(rf * 255);
        g = static_cast<uint8_t>(gf * 255);
        b = static_cast<uint8_t>(bf * 255);
    }
}