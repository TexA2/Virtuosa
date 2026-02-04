#include <viWidget.hpp>


namespace viWidget {


// ============================================================================
//                              Init функции
// ============================================================================
    std::optional<bool> viMainWidget::initMainWindow()
    {
        if (!glfwInit())
        {
            std::cerr << "Failed to initialize GLFW\n";
            return std::nullopt;
        }

        // Устанавливаем версию OpenGL
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

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
            return std::nullopt;
        }

        glfwMakeContextCurrent(window.get());

        glfwSetFramebufferSizeCallback(window.get(), resizeWindow);

        glfwSetInputMode(window.get(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);

        // ИНИЦИАЛИЗАЦИЯ GLAD
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cerr << "Failed to initialize GLAD\n";
            glfwTerminate();
            return std::nullopt;
        }

        glfwSwapInterval(0); // отключаем Vsync

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);

        glViewport(_windowSettings.width * _windowSettings.objectPanelWidth, 0, _windowSettings.width, _windowSettings.height);

        return true;
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

    void viMainWidget::initCamera() {
        viewCamera = std::make_shared<viCamera::Camera>(_windowSettings.width, _windowSettings.height);
        glfwSetWindowUserPointer(window.get(), this);
        glfwSetScrollCallback(window.get(), viCamera::Camera::mouseScrollCallback);
    }

    void viMainWidget::initShader() {
        cloudShader = std::make_shared<viShader::Shader>("../shader/ver.vs", "../shader/fragment.fs");
    }

    void viMainWidget::initCloudData() {
        cloudData = std::make_shared<viData::viManageData>();
    }

    void viMainWidget::initUI() {
        menuUI = std::make_shared<viUI::viManageUI>(cloudData, viewCamera, _windowSettings);
    }

// ============================================================================
//                              get функции
// ============================================================================
    std::shared_ptr<viData::viManageData> viMainWidget::getCloudData() {
        return cloudData;
    }

    std::shared_ptr<viUI::viManageUI> viMainWidget::getMenu() {
        return menuUI;
    }

    std::shared_ptr<viShader::Shader> viMainWidget::getShader() const {
        return cloudShader;
    }

    std::shared_ptr<viCamera::Camera> viMainWidget::getCamera() const{
        return viewCamera;
    }

// ============================================================================
// ============================================================================
    void viMainWidget::resizeWindow(GLFWwindow* window, int width, int heigth)
    {
        viMainWidget* widget = static_cast<viMainWidget*>(glfwGetWindowUserPointer(window));
        widget->_windowSettings.width = width;
        widget->_windowSettings.height = heigth;

        glViewport(width *   widget->_windowSettings.objectPanelWidth, 0, width, heigth);
    }

    void viMainWidget::render() {
        cloudShader->bind();

        glfwPollEvents();
        glClearColor(menuUI->clear_color.x,
                     menuUI->clear_color.y,
                     menuUI->clear_color.z,
                     menuUI->clear_color.w); // Устанавливаем цвет очистки

        glClear(GL_COLOR_BUFFER_BIT); //| GL_DEPTH_BUFFER_BIT);

        if (glfwGetWindowAttrib(window.get(), GLFW_ICONIFIED) != 0)
        {
            ImGui_ImplGlfw_Sleep(10);
        }

        for (auto& pair : cloudData->cloudCache)
            {
                if (pair.second->isVisible) 
                {
                    // glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
                    glBindVertexArray(pair.second->buffer.VAO);
                    
                    cloudShader->setShaderMatrix4fv("mvpMatrix", viewCamera->moveCamera(window.get()));

                    if (!menuUI->show_intensity_color)
                    {
                        cloudShader->setShader1i("useIntensityColor", menuUI->show_intensity_color);

                        glVertexAttribDivisor(1, 0);           // Отключаем инстансинг
                        glDisableVertexAttribArray(1);         // Отключаем атрибут

                        cloudShader->setShaderMatrix4f("ourColor", pair.second->point_color.x,
                                                                pair.second->point_color.y,
                                                                pair.second->point_color.z,
                                                                pair.second->point_color.w);
                    }
                    else
                    {
                        cloudShader->setShader1i("useIntensityColor", menuUI->show_intensity_color);

                        glEnableVertexAttribArray(1);
                        glVertexAttribDivisor(1, 1);
                    }
                    glDrawArraysInstanced(GL_POINTS, 0, 1, pair.second->_cloud->size());
                    glBindVertexArray(0);
                    
                    // glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    // glBindVertexArray(VAO);
                    // glActiveTexture(GL_TEXTURE0);
                    // glBindTexture(GL_TEXTURE_2D, textureBuffer); // Текстура из вашего фреймбуфера
                    // glDrawArraysInstanced(GL_POINTS, 0, 1,cloud_size);
                }
            }

        cloudShader->unbind();
    // Start the Dear ImGui frame
        menuUI->renderUI(window.get());

        glfwSwapBuffers(window.get());
    }

}


