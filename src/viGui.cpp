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
        intensity.resize(cloud_size * 3, 0.0f);
        pointPosition.resize(cloud_size);


    // Запоминаем положение и интенсивность
        float min_i = 0.f;
        float max_i = 0.f;

        for (uint i = 0; i < cloud_size; ++i)
        {
            pointPosition[i] = glm::vec3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

            min_i = std::min(min_i, cloud->points[i].intensity);
            max_i = std::max(max_i, cloud->points[i].intensity);
        }

        for (uint i = 0; i < cloud_size; ++i)
        {
            static uint pos = 0;
            float normalized_i = (cloud->points[i].intensity - min_i) / (max_i - min_i);  

            uint8_t r, g, b;
            intensityToColor(normalized_i, r, g, b);

            intensity[pos]   = r;
            intensity[++pos] = g;
            intensity[++pos] = b;

        }

        std::cout << "Razmer Cloud: "<< cloud_size << std::endl;
        cloudOpen = true;
    }



    void intensityToColor(float intensity, uint8_t& r, uint8_t& g, uint8_t& b) {

        // Нормализация интенсивности (предполагаем диапазон 0-1)
        intensity = std::max(0.0f, std::min(1.0f, intensity));
        
        // Градиент: синий (0) -> зеленый (0.5) -> красный (1)
        if (intensity < 0.5f) {
            // Синий -> Зеленый
            r = 0;
            g = static_cast<uint8_t>(2 * intensity * 255);
            b = static_cast<uint8_t>((1 - 2 * intensity) * 255);
        } else {
            // Зеленый -> Красный
            r = static_cast<uint8_t>((2 * intensity - 1) * 255);
            g = static_cast<uint8_t>((2 - 2 * intensity) * 255);
            b = 0;
        }
    }

}