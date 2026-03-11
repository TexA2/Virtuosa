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
            if (auto temp_data = _cloudData.lock())
                temp_data->newCloud();
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
                {
                    temp_cloudData->pointCloudOpen(outPath);
                    glm::vec3 centerPoint = (temp_cloudData->cloudCache[outPath]->bounds.max +
                                             temp_cloudData->cloudCache[outPath]->bounds.min );
                    centerPoint.x /= 2;
                    centerPoint.y /= 2;

                    std::cout << "centerPoint " << centerPoint.x  << " " << centerPoint.y << std::endl;

                    auto temp_camera = _viewCamera.lock();
                    temp_camera->setCameraPos(glm::vec3(centerPoint.x, centerPoint.y, 60));
                }
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
        if (ImGui::MenuItem("Save", "Ctrl+S")) {
            NFD_Init();

            nfdu8char_t *outPath;
            nfdu8filteritem_t filters[1] = { { "point cloud", "pcd" }};
            nfdsavedialognargs_t args = {0};
            args.filterList = filters;
            args.filterCount = 1;
            nfdresult_t result = NFD_SaveDialogN_With(&outPath, &args);
            if (result == NFD_OKAY)
            {
                if(auto temp_cloudData = _cloudData.lock())
                {
                    temp_cloudData->savePointCloud(selectedCloudId, std::string(outPath));
                    std::cout << "cloud save to: " << std::string(outPath) << std::endl;
                }
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
        if (ImGui::MenuItem("Save As..")) {}
        if (ImGui::MenuItem("Background Color")) {show_BackroundColor = true;}
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Alt+F4")) {
            buttonQuit_ = true;
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
                    ModeButton("Draw", Mode::drawMode, curMode, buttonSize);
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
        
        if (ImGui::Button(label, size))
        { 
            curMode = buttonMode;
            if (label == "Transform" && !showTransform_ ) showTransform_ = true;
        }


        if (isActive) ImGui::PopStyleColor(2);
    }

    void viManageUI::viewMode(GLFWwindow* window) {
        ImGuiIO& io = ImGui::GetIO();
        bool mouseOverImGui = io.WantCaptureMouse;
        
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
    }

    void viManageUI::selectMode(GLFWwindow* window) {
        // ImGuiIO& io = ImGui::GetIO();
        // if (!io.WantCaptureMouse && io.MouseClicked[0])
        // {
        //     double xpos, ypos;
        //     glm::vec3 RayOrigin;
        //     glm::vec3 RayDirection;

        //     glfwGetCursorPos(window, &xpos, &ypos);

        //     if(auto temp_camera = _viewCamera.lock())
        //     {
        //         temp_camera->rayCast(window, xpos, ypos);
        //         RayOrigin = temp_camera->rayData.RayOrigin;      // (0, 0, 60)
        //         RayDirection = temp_camera->rayData.RayDirection; // единичный вектор
        //     }

            //     glUseProgram(computeProgram);

            //     int pointCount = -1;
            //     if(auto temp_cloudData = _cloudData.lock())
            //     {
            //         pointCount = temp_cloudData->cloudCache.begin()->second->_cloud->points.size();
            //         glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0,  temp_cloudData->cloudCache.begin()->second->buffer.SSBO);
            //     }

            //     if(auto temp_camera = _viewCamera.lock())
            //     {
            //         glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, temp_camera->rayBuffer);
            //     }

            //     glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, resultBuffer);

            //     std::cout << "count " << pointCount << std::endl;

            //     GLuint numGroups = (pointCount + 255) / 256;

            // resultData = {-1, 3.40282e+38f, 0};
            // glBindBuffer(GL_SHADER_STORAGE_BUFFER, resultBuffer);
            // glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(ResultData), &resultData);


            //             if(auto temp_cloudData = _cloudData.lock()) {
            //     auto& cloud = temp_cloudData->cloudCache.begin()->second;
            //     auto& firstPoint = cloud->_cloud->points[0];
            //     std::cout << "First point: (" << firstPoint.x << ", " 
            //             << firstPoint.y << ", " << firstPoint.z << ")" << std::endl;
            // }

            //     glDispatchCompute(numGroups, 1, 1);

            //     glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


            //     ResultData result;
            //     glBindBuffer(GL_SHADER_STORAGE_BUFFER, resultBuffer);
            //     glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(ResultData), &result);


            // // Использование результатов
            // if (result.foundPoint) {
            //     printf("Найденная точка: индекс %d, расстояние %f\n", 
            //         result.selectedIndex, sqrt(result.minDistance));
            //         std::cout << "result.debugCounter " << result.debugCounter << std::endl;
            //     } else {
            //         printf("Точки не найдены\n");
            //         std::cout << "result.debugCounter " << result.debugCounter << std::endl;
            //     }
        //}
    }

    void viManageUI::drawMode(GLFWwindow* window) {
        if (auto temp_data = _cloudData.lock()) 
        {
            if (temp_data->cloudCache.empty())
            {
            std::cout << "No data" << std::endl;
            return;
            }
        }

        ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureMouse && io.MouseClicked[0])
        {
            double xpos, ypos;
            glm::vec3 RayOrigin;
            glm::vec3 RayDirection;
            float scale = 1;

            glfwGetCursorPos(window, &xpos, &ypos);

            if(auto temp_camera = _viewCamera.lock())
            {
                temp_camera->rayCast(window, xpos, ypos);
                RayOrigin = temp_camera->rayData.RayOrigin; 
                RayDirection = temp_camera->rayData.RayDirection;
            }

            std::cout << "RayOrigin " << RayOrigin.x  << " " <<  RayOrigin.y << std::endl;
            std::cout << "RayDirection " << RayDirection.x << " " << RayDirection.y << std::endl;
            std::cout  << std::endl;
            std::cout << std::endl;

            if(auto temp_cloud = _cloudData.lock())
            {
                auto cloud = temp_cloud->cloudCache[selectedCloudId]->_cloud;

                pcl::PointXYZI point;

                point.x = RayOrigin.x;
                point.y = RayOrigin.y; 
                point.z = RayOrigin.z;
                point.intensity = 1.f;

                cloud->push_back(point);
                temp_cloud->cloudCache.begin()->second->intensity.push_back(1);
                temp_cloud->cloudCache.begin()->second->intensity.push_back(1);
                temp_cloud->cloudCache.begin()->second->intensity.push_back(1);
            
                glBindBuffer(GL_ARRAY_BUFFER, temp_cloud->cloudCache[selectedCloudId]->buffer.pointVBO);
                glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, temp_cloud->cloudCache[selectedCloudId]->buffer.intensityVBO);
                glBufferData(GL_ARRAY_BUFFER, temp_cloud->cloudCache[selectedCloudId]->intensity.size() * sizeof(float), temp_cloud->cloudCache[selectedCloudId]->intensity.data(), GL_DYNAMIC_DRAW);
            }
        }
    }

    void viManageUI::transformMode(GLFWwindow* window) {

        if (showTransform_) {
        // Вся трансформация происходит относительно центра мира 0,0,0 , а не центра облака точек
        static float moveX = 0.f, moveY = 0.f, moveZ = 0.f;

        ImGui::Begin("Transform", &showTransform_);
            ImGui::Text("Move");
            ImGui::PushItemWidth(100.0f);
            ImGui::InputFloat("moveX", &moveX);
            ImGui::SameLine();
            ImGui::InputFloat("moveY", &moveY);
            ImGui::SameLine();
            ImGui::InputFloat("moveZ", &moveZ);
            if (ImGui::Button("move"))
            {
                glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), 
                                                            glm::vec3(moveX, moveY, moveZ));
                if (auto temp_data = _cloudData.lock())
                {
                    auto cloud = temp_data->cloudCache[selectedCloudId]->_cloud;

                    void* ptr = glfwGetWindowUserPointer(window);
                    if (!ptr) return;

                    viWidget::viMainWidget* widget = static_cast<viWidget::viMainWidget*>(ptr);
                    widget->getShader()->computeTransform(translationMatrix, cloud->size());

                    temp_data->readComputeData(cloud->size(), selectedCloudId);
                    }

                    glBindBuffer(GL_ARRAY_BUFFER, temp_data->cloudCache.begin()->second->buffer.pointVBO);
                    glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->data(), GL_DYNAMIC_DRAW);
                }
            }
            ImGui::PopItemWidth();


            static float scaleX = 1.f, scaleY = 1.f, scaleZ = 1.f;

            ImGui::Text("Scaling");
            ImGui::PushItemWidth(100.0f);
            ImGui::InputFloat("scaleX", &scaleX);
            ImGui::SameLine();
            ImGui::InputFloat("scaleY", &scaleY);
            ImGui::SameLine();
            ImGui::InputFloat("scaleZ", &scaleZ);
            if (ImGui::Button("scale"))
            {
                glm::mat4 scalingMatrix = glm::scale(glm::mat4(1.0f), 
                                            glm::vec3(scaleX, scaleY, scaleZ));

                if (auto temp_data = _cloudData.lock())
                {
                    auto cloud = temp_data->cloudCache[selectedCloudId]->_cloud;
                    for (auto &point : *cloud)
                    {
                        glm::vec4 pointV (point.x, point.y, point.z, 1.f);
                        pointV = scalingMatrix * pointV;
                        point.x = pointV.x;
                        point.y = pointV.y;
                        point.z = pointV.z;
                    }

                    glBindBuffer(GL_ARRAY_BUFFER, temp_data->cloudCache.begin()->second->buffer.pointVBO);
                    glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->data(), GL_DYNAMIC_DRAW);
                }
            }
            ImGui::PopItemWidth();


        ImGui::End();
        }
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

        //TODO: Баг если открыто окно и снять select с облака точек, то программа умерт
        if(show_pointColor)
        {
            ImGui::Begin("Point Cloud COlor Render", &show_pointColor);
            if (auto temp_cloudData = _cloudData.lock())
                ImGui::ColorEdit3("points color", (float*)&temp_cloudData->cloudCache[selectedCloudId]->point_color);
            ImGui::End();
        }

        if(buttonQuit_) {
            glfwSetWindowShouldClose(window, true);
        }

        // Режим работы и сигналы от мышки
        switch (curMode)
        {
        case Mode::viewMode:
            viewMode(window);
            break;
        case Mode::selectMode:
            selectMode(window);
            break;
        case Mode::drawMode:
            drawMode(window);
            break;
        case Mode::transformMode:
            transformMode(window);
            break;
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

}