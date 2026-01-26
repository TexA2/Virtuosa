#ifndef VI_UI
#define VI_UI

#include <viCamera.hpp>
#include <viData.hpp>
#include <cstdio>
#include <memory>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "nfd.hpp"


namespace viUI {

    class viManageUI {
        public:
            viManageUI(std::shared_ptr<viData::viManageData> &cloudData,
                        std::shared_ptr<viCamera::Camera> &viewCamera) :
                        _cloudData(cloudData),
                        _viewCamera(viewCamera){}

            ~viManageUI() = default;


            void ShowExampleAppMainMenuBar();
            void ShowExampleMenuFile();
        
        private:
            std::weak_ptr<viData::viManageData> _cloudData;
            std::weak_ptr<viCamera::Camera> _viewCamera;



    };
}

#endif