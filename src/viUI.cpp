#include <viUI.hpp>
#include "viWidget.hpp" 

namespace viUI {

    void viManageUI::ShowExampleAppMainMenuBar() {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
                ShowExampleMenuFile();
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Tools"))
            {
                if (ImGui::MenuItem("Projection")) { 
                    if (auto temp_camera = _viewCamera.lock())    
                        temp_camera->toggleProjection(); 
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
    }

    void viManageUI::ShowExampleMenuFile() {
        if (ImGui::MenuItem("New")) 
        { 
            //TODO: поченить после изменений
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
                if(auto temp_cloudData = _cloudData.lock())
                    temp_cloudData->pointCloudOpen(outPath);
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
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4")) {
            buttonQuit = true;
        }
    }

    void viManageUI::showObjectPanel() {

        float menuBarHeight = ImGui::GetFrameHeight();

        float renderAreaHeight = _windowsSetting.height - menuBarHeight;
        ImGui::SetNextWindowPos(ImVec2(0, menuBarHeight));
        ImGui::SetNextWindowSize(ImVec2(_windowsSetting.width * _windowsSetting.objectPanelWidth,
                                                renderAreaHeight));

        ImGui::Begin("Objects Panel", nullptr, 
            ImGuiWindowFlags_NoTitleBar | 
            ImGuiWindowFlags_NoResize | 
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse);


        // === ВЕРХНЯЯ ОБЛАСТЬ (25% высоты) ===
            float topHeight = ImGui::GetContentRegionAvail().y * 0.25f;

            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));

            ImGui::BeginChild("Top Region", ImVec2(0, topHeight), true, 
                                            ImGuiWindowFlags_NoScrollbar);
            {

            ImGui::PopStyleColor();

            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));

            ImGui::Text("Render Objects");
            ImGui::Separator();

            if (auto temp_cloudData = _cloudData.lock())
                for (const auto& pair : temp_cloudData->cloudCache)
                {
                    ImGui::PushID(&pair);

                    bool isSelected = (selectedCloudId == pair.first);
                    pair.second->isSelected = isSelected;

                    if (ImGui::Selectable(pair.first.c_str(), isSelected)) 
                    {
                        if (isSelected)
                        {
                            selectedCloudId.clear();
                            pair.second->isSelected = false;
                        }
                        else
                        {
                            selectedCloudId = pair.first;
                            for (auto& otherPair : temp_cloudData->cloudCache)
                                otherPair.second->isSelected = (otherPair.first == pair.first);
                        }
                    }

                    ImGui::PopID();
                }

            }
            ImGui::PopStyleColor();
            ImGui::EndChild();

        // == ЦЕНТРАЛЬНАЯ ОБЛАСТЬ (50%)
            float middleHeight = ImGui::GetContentRegionAvail().y * 0.50f;
            if (ImGui::BeginChild("Mid Region", ImVec2(0, middleHeight), true,
                ImGuiWindowFlags_NoScrollbar) )
            {
                ImVec2 buttonSize = ImVec2(100, 64);
                ImGui::BeginGroup();
                    ModeButton("View", Mode::viewMode, curMode, buttonSize);
                    ModeButton("Select", Mode::selectMode, curMode, buttonSize);
                    ModeButton("Transform", Mode::transformMode, curMode, buttonSize);
                ImGui::EndGroup();
            }
            ImGui::EndChild();
        

        // === НИЖНЯЯ ОБЛАСТЬ (25% высоты) ===
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
            if (ImGui::BeginChild("Bot Region", ImVec2(0, 0), true,
                ImGuiWindowFlags_NoScrollbar) )
            {
                ImGui::PopStyleColor();

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
                ImGui::Text("Setting Objects");
                ImGui::Separator();

                if (!selectedCloudId.empty())
                {
                    if (auto temp_cloudData = _cloudData.lock())
                    {
                        ImGui::Checkbox("Visible", &temp_cloudData->cloudCache[selectedCloudId]->isVisible);
                        ImGui::Checkbox("Intensity color", &temp_cloudData->cloudCache[selectedCloudId]->intensityColor); 

                        if (ImGui::Button("Color Setting")) show_pointColor = !show_pointColor;
                    }
                }


            }
            ImGui::PopStyleColor();
            ImGui::EndChild();

        ImGui::End();
    }


    void viManageUI::ModeButton(const char* label, Mode buttonMode, Mode& curMode, const ImVec2& size) {
        bool isActive = (curMode == buttonMode);

        if (isActive)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.6f, 0.3f, 1.0f));
        }
        
        if (ImGui::Button(label, size)) curMode = buttonMode;
        
        if (isActive) ImGui::PopStyleColor(2);

    }


    void viManageUI::renderUI(GLFWwindow* window) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        ShowExampleAppMainMenuBar();
        showObjectPanel();

        if (show_BackroundColor)
        {
            ImGui::Begin("Background Color Render", &show_BackroundColor);
            ImGui::ColorEdit3("clear color", (float*)&clear_color);
            ImGui::End();
        }


        if(show_pointColor)
        {
            ImGui::Begin("Point Cloud COlor Render", &show_pointColor);
            if (auto temp_cloudData = _cloudData.lock())
                ImGui::ColorEdit3("points color", (float*)&temp_cloudData->cloudCache[selectedCloudId]->point_color);
            ImGui::End();
        }


        if(buttonQuit) {
            glfwSetWindowShouldClose(window, true);
        }



        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // 1. Получаем состояние мыши
        ImGuiIO& io = ImGui::GetIO();
        bool mouseOverImGui = io.WantCaptureMouse;
        
        // 2. Проверяем нажатие левой кнопки мыши
        static bool wasMousePressed = false;
        int mouseState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
        bool isMousePressed = (mouseState == GLFW_PRESS);


        if (!mouseOverImGui)
        {
            if (isMousePressed)
            {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);

                if(auto temp_camera = _viewCamera.lock())
                    temp_camera->mouseMoveCallback(window,  xpos,  ypos);

            } else {
                if(auto temp_camera = _viewCamera.lock()) 
                    temp_camera->resetFirstMouse(); }
        }

        std::cout << "Current Mode " << static_cast<int>(curMode) << std::endl;
    }
}