#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono> 


#include <viShader.hpp>
#include <viCamera.hpp>
#include <viWidget.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "nfd.hpp"

#define WIDTH  1280
#define HEIGHT 1024

// Helper to wire demo markers located in code to an interactive browser
typedef void (*ImGuiDemoMarkerCallback)(const char* file, int line, const char* section, void* user_data);
extern ImGuiDemoMarkerCallback      GImGuiDemoMarkerCallback;
extern void*                        GImGuiDemoMarkerCallbackUserData;
ImGuiDemoMarkerCallback             GImGuiDemoMarkerCallback = NULL;
void*                               GImGuiDemoMarkerCallbackUserData = NULL;

#define IMGUI_DEMO_MARKER(section)  do { if (GImGuiDemoMarkerCallback != NULL) GImGuiDemoMarkerCallback("imgui_demo.cpp", __LINE__, section, GImGuiDemoMarkerCallbackUserData); } while (0)



ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 point_color = ImVec4(1.f, 1.f, 0.f, 1.00f);



myCamera::Camera viewCamera;
float deltaTime = 0.0f;	// время между текущим и последним кадрами
float lastFrame = 0.0f; // время последнего кадра


bool  initCloud = false;
std::vector<float> intensity;  //(cloud_size * 3, 0.0f);
std::vector<glm::vec3> pointPosition; // массив который содержит точки
bool show_intensity_color = false;
// bool buttonQuit = false;


unsigned int framebuffer;
unsigned int textureBuffer;

void createFrameBuffer() {
    // unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);


    //создание текстуры рендеринга
    // unsigned int textureBuffer;
    glGenTextures(1, &textureBuffer);
    glBindTexture(GL_TEXTURE_2D, textureBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureBuffer, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "Framebuffer не готов!" << std::endl;
    }

     glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


void mouseScrollCallbacl(GLFWwindow* window, double xoffset, double yoffset){
    viewCamera.zoomCamera(yoffset, deltaTime);
}


int main() {

    viWidget::viMainWidget MainWindow;
    
    GLFWwindow* window = MainWindow.initMainWindow();

    if (!window)
    {
        std::cout << "Error init window" << std::endl;
        return -1;
    }

    // придумать, как запихать эти колбеки гармонично
    glfwSetWindowUserPointer(window, &viewCamera);
    glfwSetScrollCallback(window, mouseScrollCallbacl);

    MainWindow.initGui();
    
    createFrameBuffer(); //В данный момент рудимент, но скоро нужно будет использовать

// настройка вершинных буферов
    unsigned int VAO, instanceVBO;
    unsigned int intensityVBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &instanceVBO); 
    glGenBuffers(1, &intensityVBO);

    glBindVertexArray(VAO);


    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, pointPosition.size() * sizeof(glm::vec3), pointPosition.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribDivisor(0, 1); 


    glBindBuffer(GL_ARRAY_BUFFER, intensityVBO);
    glBufferData(GL_ARRAY_BUFFER, intensity.size() * sizeof(float), intensity.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    glVertexAttribDivisor(1, 1);


 //отвязка параметров чтоб случайно не изменить   
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


    myShader::Shader First ("../shader/ver.vs", "../shader/fragment.fs");

    First.use();

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.f);

    glm::mat4 projection;
    projection = glm::perspective(glm::radians(75.f), float(WIDTH) / float(HEIGHT), 0.1f, 1000.f);
    //projection = glm::ortho(0.0f, float(WIDTH), 0.0f, float(HEIGHT), 0.1f, 1000.0f);


    unsigned int modelLoc = glGetUniformLocation(First.ID, "model");
    unsigned int viewLoc = glGetUniformLocation(First.ID, "view");
    unsigned int projectionLoc = glGetUniformLocation(First.ID, "projection");
    unsigned int colorLoc = glGetUniformLocation(First.ID, "ourColor");

    unsigned int useIntensityColorLoc = glGetUniformLocation(First.ID, "useIntensityColor");


    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));

    viewCamera.setWorldModel(&model, modelLoc);    

    double lastTime = glfwGetTime();
    int frameCount = 0;

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);


// ****Основной цикл****
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();


        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w); // Устанавливаем цвет очистки
        glClear(GL_COLOR_BUFFER_BIT); //| GL_DEPTH_BUFFER_BIT);


        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;  


        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }


        if(viWidget::cloudOpen)
        {
            if(!initCloud)
            {
                glBindVertexArray(VAO);
                glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
                glBufferData(GL_ARRAY_BUFFER, pointPosition.size() * sizeof(glm::vec3), pointPosition.data(), GL_STATIC_DRAW);

                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
                glEnableVertexAttribArray(0);
                glVertexAttribDivisor(0, 1); 


                glBindBuffer(GL_ARRAY_BUFFER, intensityVBO);
                glBufferData(GL_ARRAY_BUFFER, intensity.size() * sizeof(float), intensity.data(), GL_STATIC_DRAW);
                
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
                glEnableVertexAttribArray(1);
                glVertexAttribDivisor(1, 1);

                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
                initCloud = true;

                viewCamera.resetToZero();
            }
            // glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

            glViewport(0, 0, WIDTH, HEIGHT);

            // // Очищаем буферы
            // glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            // glClear(GL_COLOR_BUFFER_BIT);

            glBindVertexArray(VAO);
            
            view = viewCamera.moveCamera(window, deltaTime);
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

            if (!show_intensity_color)
            {
                glUniform1i(useIntensityColorLoc, show_intensity_color);

                glVertexAttribDivisor(1, 0);           // Отключаем инстансинг
                glDisableVertexAttribArray(1);         // Отключаем атрибут

                glUniform4f(colorLoc,  point_color.x, point_color.y, point_color.z, point_color.w);
            }
            else
            {
                glUniform1i(useIntensityColorLoc, show_intensity_color);

                glEnableVertexAttribArray(1);
                glVertexAttribDivisor(1, 1);
            }
            glDrawArraysInstanced(GL_POINTS, 0, 1, viWidget::cloud_size);
            glBindVertexArray(0);
            
            // glBindFramebuffer(GL_FRAMEBUFFER, 0);
            // glBindVertexArray(VAO);
            // glActiveTexture(GL_TEXTURE0);
            // glBindTexture(GL_TEXTURE_2D, textureBuffer); // Текстура из вашего фреймбуфера
            // glDrawArraysInstanced(GL_POINTS, 0, 1,cloud_size);
        }


        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        // glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Устанавливаем цвет очистки
        // glClear(GL_COLOR_BUFFER_BIT); //| GL_DEPTH_BUFFER_BIT);


        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        viWidget::ShowExampleAppMainMenuBar(pointPosition, intensity);


        if (viWidget::show_BackroundColor)
        {
            ImGui::Begin("Background Color Render", &viWidget::show_BackroundColor);
            
            ImGui::ColorEdit3("clear color", (float*)&clear_color);

            ImGui::End();
        }


        if(viWidget::show_pointColor)
        {
            ImGui::Begin("Point Cloud COlor Render", &viWidget::show_pointColor);
            
            ImGui::ColorEdit3("clear color", (float*)&point_color);
            ImGui::Checkbox("Intensity", &show_intensity_color);  

            ImGui::End();
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
                    viewCamera.mouseMoveCallback(window,  xpos,  ypos);
            } else { viewCamera.resetFirstMouse(); }
        }


        glfwSwapBuffers(window);

        //Счетчик FPS
        double currentTime = glfwGetTime();
        frameCount++;
        
        if (currentTime - lastTime >= 1.0)
        {
            std::cout << "FPS: " << frameCount << std::endl;
            frameCount = 0;
            lastTime = currentTime;
        }
    
        if(viWidget::buttonQuit) {
            glfwSetWindowShouldClose(window, true);
        }
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);

    glfwTerminate();
    return 0;
}
