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
        if (ImGui::MenuItem("Point Color")) {   show_pointColor = true;}
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

        ImGui::Text("Render Objects");
        ImGui::Separator();

        ImGui::End();
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

            if(auto temp_cloudData = _cloudData.lock())
                ImGui::ColorEdit3("clear color", (float*)&temp_cloudData->viCloud.point_color);

            ImGui::Checkbox("Intensity", &show_intensity_color);  
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
    }
}