#include <viData.hpp>

namespace viData {

    std::string viManageData::getFileName(const std::string& path) {
        std::filesystem::path obj(path);
        return obj.filename().string();
    }

    void detectFormat(std::string path) {
        pcl::PCLPointCloud2 cloud_blob;


        if (pcl::io::loadPCDFile(path, cloud_blob) == -1)
            std::cerr << "Ошибка: не удалось загрузить файл " << path << std::endl;

        for (const auto& field : cloud_blob.fields) {
            std::cout << field.name << " offset " << field.offset << std::endl;
        }


    }

    void viManageData::pointCloudOpen(std::string path) {

        detectFormat(path);

        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
        pcl::io::loadPCDFile(path, *cloud);
        std::string name = path;

        if (cloudCache.find(name) == cloudCache.end())
        {
            std::shared_ptr<CloudData> temp_cloud = std::make_shared<CloudData> ();
            temp_cloud->_cloud = cloud;

            calculateCloudBounds(temp_cloud);

            for (uint i = 0; i < temp_cloud->cloud_size(); ++i)
            {
                float normalized_i = (temp_cloud->_cloud->at<float>(i, 12) 
                                    - temp_cloud->bounds.cloudIntensity.x)
                                    / (temp_cloud->bounds.cloudIntensity.y - temp_cloud->bounds.cloudIntensity.x);  

                float r, g, b;
                intensityToColor(normalized_i, r, g, b);

                temp_cloud->intensity.push_back(r);
                temp_cloud->intensity.push_back(g);
                temp_cloud->intensity.push_back(b);
            }
            cloudCache[name] = temp_cloud;
        } else 
        {
            // TODO :: Делать имя: имя_copy
            std::cout << "Object ush dobavlen " << std::endl;
        }
    }


    void viManageData::calculateCloudBounds(std::shared_ptr<CloudData> cloud) {
        
        for (uint i = 0; i < cloud->cloud_size(); ++i)
        {
            float x = cloud->_cloud->at<float>(i, 0);
            float y = cloud->_cloud->at<float>(i, 4);
            float z = cloud->_cloud->at<float>(i, 8);
            float intensity = cloud->_cloud->at<float>(i, 12);

            cloud->bounds.min.x = std::min(cloud->bounds.min.x, x);
            cloud->bounds.min.y = std::min(cloud->bounds.min.y, y);
            cloud->bounds.min.z = std::min(cloud->bounds.min.z, z);

            cloud->bounds.max.x = std::max(cloud->bounds.max.x, x);
            cloud->bounds.max.y = std::max(cloud->bounds.max.y, y);
            cloud->bounds.max.z = std::max(cloud->bounds.max.z, z);

            cloud->bounds.cloudIntensity.x = std::min(cloud->bounds.cloudIntensity.x, intensity);
            cloud->bounds.cloudIntensity.y = std::max(cloud->bounds.cloudIntensity.y, intensity);
        }
    }

    void viManageData::cloudBuffer(std::shared_ptr<CloudData> cloud) {

        glGenVertexArrays(1, &cloud->buffer.VAO);
        glGenBuffers(1, &cloud->buffer.pointVBO); 
        glGenBuffers(1, &cloud->buffer.intensityVBO);

        glBindVertexArray(cloud->buffer.VAO);

        glBindBuffer(GL_ARRAY_BUFFER, cloud->buffer.pointVBO);
        glBufferData(GL_ARRAY_BUFFER, cloud->_cloud->data.size(), cloud->_cloud->data.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 4 , (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribDivisor(0, 1); 

        glBindBuffer(GL_ARRAY_BUFFER, cloud->buffer.intensityVBO);
        glBufferData(GL_ARRAY_BUFFER, cloud->intensity.size() * sizeof(float), cloud->intensity.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);

        glVertexAttribDivisor(1, 1);


        //отвязка параметров чтоб случайно не изменить   
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);


        //Создадим буффер SSBO
        glGenBuffers(1, &cloud->buffer.SSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, cloud->buffer.SSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, 
                    cloud->cloud_size() * sizeof(uint8_t),
                    cloud->_cloud->data.data(),                       
                    GL_DYNAMIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, cloud->buffer.SSBO);
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


    void viManageData::newCloud(uint8_t type) {

        using CloudVariant = std::variant<
            pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr>;

        CloudVariant temp_cloud;

        switch (type)
        {
        case 1:
            {
                temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                break;
            }
        case 2:
            {
                temp_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                break;
            }
        case 3:
            {
                temp_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                break;
            }
        }

        pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2());

        std::visit([&cloud2](auto& cloud){
            cloud->width = 1;
            pcl::toPCLPointCloud2(*cloud, *cloud2);
        }, temp_cloud);

        std::string name ("new_cloud");

        std::shared_ptr<CloudData> temp_cloudData = std::make_shared<CloudData> ();
        temp_cloudData->_cloud = cloud2;
        cloudBuffer(temp_cloudData);
        cloudCache[name] = temp_cloudData;
    }

    void viManageData::savePointCloud(std::string nameCloud, std::string path) {
        // pcl::io::savePCDFileASCII (path, *(cloudCache[nameCloud]->_cloud));
        // Для Интенсивности 
        // провекру через поля
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(*cloudCache[nameCloud]->_cloud, *cloud);
        pcl::io::savePCDFileASCII(path, *cloud);
        std::cout << "Saved cloud as XYZI to: " << path << std::endl;
    }


    void viManageData::readComputeData(std::string select) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, cloudCache[select]->buffer.SSBO);
        glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 
                       cloudCache[select]->cloud_size() * sizeof(uint8_t), 
                       cloudCache[select]->_cloud->data.data());
    }
}