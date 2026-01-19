#include <viData.hpp>

namespace viData {

    void viManageData::pointCloudOpen(std::string path) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud);

        viCloud._cloud = cloud;

        calculateCloudBounds();

        for (uint i = 0; i < viCloud._cloud->size(); ++i)
        {
            float normalized_i = (viCloud._cloud->points[i].intensity - viCloud.bounds.cloudIntensity.x)
                                 / (viCloud.bounds.cloudIntensity.y- viCloud.bounds.cloudIntensity.x);  

            float r, g, b;
            intensityToColor(normalized_i, r, g, b);

            viCloud.intensity.push_back(r);
            viCloud.intensity.push_back(g);
            viCloud.intensity.push_back(b);
        }
        cloudBuffer();
    }


    void viManageData::calculateCloudBounds() {
        
        for (const auto& p : viCloud._cloud->points) 
        {
            viCloud.bounds.min.x = std::min(viCloud.bounds.min.x, p.x);
            viCloud.bounds.min.y = std::min(viCloud.bounds.min.y, p.y);
            viCloud.bounds.min.z = std::min(viCloud.bounds.min.z, p.z);

            viCloud.bounds.max.x = std::max(viCloud.bounds.max.x, p.x);
            viCloud.bounds.max.y = std::max(viCloud.bounds.max.y, p.y);
            viCloud.bounds.max.z = std::max(viCloud.bounds.max.z, p.z);

            viCloud.bounds.cloudIntensity.x = std::min(viCloud.bounds.cloudIntensity.x, p.intensity);
            viCloud.bounds.cloudIntensity.y = std::max(viCloud.bounds.cloudIntensity.y, p.intensity);
        }
    }

    void viManageData::cloudBuffer() {

        glGenVertexArrays(1, &viCloud.buffer.VAO);
        glGenBuffers(1, &viCloud.buffer.pointVBO); 
        glGenBuffers(1, &viCloud.buffer.intensityVBO);

        glBindVertexArray(viCloud.buffer.VAO);

        glBindBuffer(GL_ARRAY_BUFFER, viCloud.buffer.pointVBO);
        glBufferData(GL_ARRAY_BUFFER, viCloud._cloud->size() * sizeof(pcl::PointXYZI), viCloud._cloud->data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZI), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribDivisor(0, 1); 

        glBindBuffer(GL_ARRAY_BUFFER, viCloud.buffer.intensityVBO);
        glBufferData(GL_ARRAY_BUFFER, viCloud.intensity.size() * sizeof(float), viCloud.intensity.data(), GL_STATIC_DRAW);

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