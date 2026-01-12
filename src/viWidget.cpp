#include <viWidget.hpp>



namespace viWidget {

    GLFWwindow* viMainWidget::initMainWindow()
    {
        if (!glfwInit())
        {
            std::cerr << "Failed to initialize GLFW\n";
            return nullptr;
        }

        // Устанавливаем версию OpenGL
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window = glfwCreateWindow(_width, _height, "Virtuosa", NULL, NULL);

        if (!window)
        {
            std::cerr << "Failed to open GLFW window\n";
            glfwTerminate();
            return nullptr;
        }

        glfwMakeContextCurrent(window);

        glfwSetFramebufferSizeCallback(window, resizeWindow);

        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

        // ИНИЦИАЛИЗАЦИЯ GLAD
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cerr << "Failed to initialize GLAD\n";
            glfwTerminate();
            return nullptr;
        }

        glfwSwapInterval(0); // отключаем Vsync

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);

        glViewport(0, 0, _width, _height);

        return window;
    }

    void viMainWidget::initGui()
    {
        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

        // Setup Dear ImGui style
        ImGui::StyleColorsDark();

        float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());


        // Setup scaling
        ImGuiStyle& style = ImGui::GetStyle();
        style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
        style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(window, true);

        const char* glsl_version = "#version 450";

        ImGui_ImplOpenGL3_Init(glsl_version);
    }







    void viMainWidget::resizeWindow(GLFWwindow* window, int width, int heigth)
    {
        glViewport(0, 0, width, heigth);
    }





    bool show_BackroundColor = false;
    bool show_pointColor = false;
    bool cloudOpen = false;
    int cloud_size = 0;

    bool buttonQuit = false;

    void ShowExampleMenuFile(std::vector<glm::vec3>& pointPosition, std::vector<float>& intensity)
    {
        if (ImGui::MenuItem("New")) 
        { 
            // очищаем массив точек
            pointPosition.clear();
            initCloud = false;
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
            initCloud = false;
        }
        if (ImGui::MenuItem("Save", "Ctrl+S")) {}
        if (ImGui::MenuItem("Save As..")) {}
        if (ImGui::MenuItem("Background Color")) {show_BackroundColor = true;}
        if (ImGui::MenuItem("Point Color")) {   show_pointColor = true;}
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4")) {
            // делаем завершение программы
            buttonQuit = true;
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