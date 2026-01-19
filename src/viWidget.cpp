#include <viWidget.hpp>


namespace viWidget {

    bool show_BackroundColor = false;
    bool show_pointColor = false;
    bool buttonQuit = false;

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

        window = glfwCreateWindow(_windowSettings.width,
                                  _windowSettings.height,
                                  _windowSettings.title.c_str(), NULL, NULL);

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

        glViewport(0, 0, _windowSettings.width, _windowSettings.height);

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

    void viMainWidget::ShowExampleAppMainMenuBar(bool& projType) {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
                ShowExampleMenuFile();
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Tools"))
            {
                if (ImGui::MenuItem("Projection")) { projType = !projType; }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
    }

    void viMainWidget::ShowExampleMenuFile() {
        if (ImGui::MenuItem("New")) 
        { 
            viCloud._cloud->clear();
            viCloud.cloudOpen = false;
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
                viPointcloudOpen(outPath);
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
            buttonQuit = true;
        }
    }


    void viMainWidget::viPointcloudOpen(std::string path) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud);

        viCloud._cloud = cloud;

    // Запоминаем положение и интенсивность
        calculateCloudBounds();

        for (uint i = 0; i < viCloud._cloud->size(); ++i)
        {
            float normalized_i = (viCloud._cloud->points[i].intensity - viCloud.cloudIntensityMin)
                                 / (viCloud.cloudIntensityMax - viCloud.cloudIntensityMin);  

            float r, g, b;
            intensityToColor(normalized_i, r, g, b);

            viCloud.intensity.push_back(r);
            viCloud.intensity.push_back(g);
            viCloud.intensity.push_back(b);
        }

        cloudBuffer();
        viCloud.cloudOpen = true;
    }

    void viMainWidget::cloudBuffer() {

            glGenVertexArrays(1, &viCloud.VAO);
            glGenBuffers(1, &viCloud.instanceVBO); 
            glGenBuffers(1, &viCloud.intensityVBO);

            glBindVertexArray(viCloud.VAO);


            glBindBuffer(GL_ARRAY_BUFFER, viCloud.instanceVBO);
            glBufferData(GL_ARRAY_BUFFER, viCloud._cloud->size() * sizeof(pcl::PointXYZI), viCloud._cloud->data(), GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZI), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribDivisor(0, 1); 


            glBindBuffer(GL_ARRAY_BUFFER, viCloud.intensityVBO);
            glBufferData(GL_ARRAY_BUFFER, viCloud.intensity.size() * sizeof(float), viCloud.intensity.data(), GL_STATIC_DRAW);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(1);

            glVertexAttribDivisor(1, 1);


        //отвязка параметров чтоб случайно не изменить   
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
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


    void viMainWidget::calculateCloudBounds() {
        
        for (const auto& p : viCloud._cloud->points) {
            if (p.x < viCloud.cloudMinX) viCloud.cloudMinX = p.x;
            if (p.x > viCloud.cloudMaxX) viCloud.cloudMaxX = p.x;
            if (p.y < viCloud.cloudMinY) viCloud.cloudMinY = p.y;
            if (p.y > viCloud.cloudMaxY) viCloud.cloudMaxY = p.y;
            if (p.z < viCloud.cloudMinZ) viCloud.cloudMinZ = p.z;
            if (p.z > viCloud.cloudMaxZ) viCloud.cloudMaxZ = p.z;

            viCloud.cloudIntensityMin = std::min(viCloud.cloudIntensityMin, p.intensity);
            viCloud.cloudIntensityMax = std::max(viCloud.cloudIntensityMax, p.intensity);
        }
    }


    void viMainWidget::initCamera() {
        viewCamera = (new viCamera::Camera(_windowSettings.width, _windowSettings.height));
        glfwSetWindowUserPointer(window, viewCamera);
        glfwSetScrollCallback(window, viCamera::Camera::mouseScrollCallback);
    }

    viCamera::Camera* viMainWidget::getCamera() {
        return viewCamera;
    }


}


