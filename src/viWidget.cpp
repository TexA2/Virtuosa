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

        // window = glfwCreateWindow(_windowSettings.width,
        //                           _windowSettings.height,
        //                           _windowSettings.title.c_str(), NULL, NULL);

        GLFWwindow* rawWindow = glfwCreateWindow(
                                _windowSettings.width,
                                _windowSettings.height,
                                _windowSettings.title.c_str(), 
                                NULL, 
                                NULL
                                );


        window = std::shared_ptr<GLFWwindow>(
                                        rawWindow,
                                        [](GLFWwindow* w) {
                                            if (w) {
                                                glfwDestroyWindow(w);
                                            }
                                        }
                                    );


        if (!window.get())
        {
            std::cerr << "Failed to open GLFW window\n";
            glfwTerminate();
            return nullptr;
        }

        glfwMakeContextCurrent(window.get());

        glfwSetFramebufferSizeCallback(window.get(), resizeWindow);

        glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);

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

        return window.get();
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
        ImGui_ImplGlfw_InitForOpenGL(window.get(), true);

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
            //viCloud._cloud->clear();
            //viCloud.cloudOpen = false;
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
                cloudData->pointCloudOpen(outPath);
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

    void viMainWidget::initCamera() {
        viewCamera = std::make_shared<viCamera::Camera>(_windowSettings.width, _windowSettings.height);
        glfwSetWindowUserPointer(window.get(), viewCamera.get());
        glfwSetScrollCallback(window.get(), viCamera::Camera::mouseScrollCallback);
    }

    std::shared_ptr<viCamera::Camera> viMainWidget::getCamera() const{
        return viewCamera;
    }

    void viMainWidget::initShader() {
        cloudShader = std::make_shared<viShader::Shader>("../shader/ver.vs", "../shader/fragment.fs");
    }

    std::shared_ptr<viShader::Shader> viMainWidget::getShader() const {
        return cloudShader;
    }


    void viMainWidget::initCloudData() {
        cloudData = std::make_shared<viData::viManageData>();
    }

    std::shared_ptr<viData::viManageData> viMainWidget::getCloudData() {
        return cloudData;
    }

}


