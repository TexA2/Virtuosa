#include <viUI.hpp>

namespace viUI {


    bool show_BackroundColor = false;
    bool show_pointColor = false;
    bool buttonQuit = false;

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
}