#include <viGui.hpp>



namespace viGui {

    bool show_BackroundColor = false;
    bool show_pointColor = false;
    bool cloudOpen = false;
    int cloud_size = 0;

    void ShowExampleMenuFile(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity)
    {
        if (ImGui::MenuItem("New")) 
        { 

            // очищаем массив точек
        }


        if (ImGui::MenuItem("Open", "Ctrl+O")) {
            NFD_Init();

            nfdu8char_t *outPath;
            nfdu8filteritem_t filters[1] = { { "point cloud", "pcd" }};
            nfdopendialogu8args_t args = {0};
            args.filterList = filters;
            args.filterCount = 1;
            nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
            if (result == NFD_OKAY)
            {
                pointCloudOpen(outPath, pointPosition, intensity);
            }
            else if (result == NFD_CANCEL)
            {
                puts("User pressed cancel.");
            }
            else 
            {
                printf("Error: %s\n", NFD_GetError());
            }

            NFD_Quit();
        }
        if (ImGui::MenuItem("Save", "Ctrl+S")) {}
        if (ImGui::MenuItem("Save As..")) {}
        if (ImGui::MenuItem("Background Color")) {show_BackroundColor = true;}
        if (ImGui::MenuItem("Point Color")) {   show_pointColor = true;}
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4")) {
            // делаем завершение программы
        }
    }




    void ShowExampleAppMainMenuBar(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity)
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
                ShowExampleMenuFile(pointPosition, intensity);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
    }


    void pointCloudOpen(std::string path, std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity){
    // откроем pcd файл
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        
        pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud);

        cloud_size = cloud->width * cloud->height;

        intensity.clear();


        pointPosition.clear();
        pointPosition.resize(cloud_size);


    // Запоминаем положение и интенсивность
        float min_i = std::numeric_limits<float>::max();
        float max_i = std::numeric_limits<float>::lowest();

        for (uint i = 0; i < cloud_size; ++i)
        {
            pointPosition[i] = glm::vec3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

            min_i = std::min(min_i, cloud->points[i].intensity);
            max_i = std::max(max_i, cloud->points[i].intensity);
        }

        for (uint i = 0; i < cloud_size; ++i)
        {
            float normalized_i = (cloud->points[i].intensity - min_i) / (max_i - min_i);  

            float r, g, b;
            intensityToColor(normalized_i, r, g, b);

            intensity.push_back(r);
            intensity.push_back(g);
            intensity.push_back(b);

        }

        std::cout << "Razmer Cloud: "<< cloud_size << std::endl;
        cloudOpen = true;
    }

    void intensityToColor(float intensity, float& r, float& g, float& b) {
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