#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono> 


#include <myShader.hpp>
#include <myCamera.hpp>
#include <viGui.hpp>

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


ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
ImVec4 point_color = ImVec4(1.f, 1.f, 0.f, 1.00f);


// Helper to wire demo markers located in code to an interactive browser
typedef void (*ImGuiDemoMarkerCallback)(const char* file, int line, const char* section, void* user_data);
extern ImGuiDemoMarkerCallback      GImGuiDemoMarkerCallback;
extern void*                        GImGuiDemoMarkerCallbackUserData;
ImGuiDemoMarkerCallback             GImGuiDemoMarkerCallback = NULL;
void*                               GImGuiDemoMarkerCallbackUserData = NULL;

#define IMGUI_DEMO_MARKER(section)  do { if (GImGuiDemoMarkerCallback != NULL) GImGuiDemoMarkerCallback("imgui_demo.cpp", __LINE__, section, GImGuiDemoMarkerCallbackUserData); } while (0)

myCamera::Camera viewCamera;
float deltaTime = 0.0f;	// время между текущим и последним кадрами
float lastFrame = 0.0f; // время последнего кадра


bool  initCloud = false;
std::vector<float> intensity;  //(cloud_size * 3, 0.0f);
std::vector<glm::vec3> pointPosition; // массив который содержит точки


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


void frambuffer_size_callback(GLFWwindow* window, int width, int heigth) {
    glViewport(0, 0, width, heigth);
}


void mouseScrollCallbacl(GLFWwindow* window, double xoffset, double yoffset){
    viewCamera.zoomCamera(yoffset, deltaTime);
}


GLFWwindow* init() {
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return nullptr;
    }

    // Устанавливаем версию OpenGL
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Simple GLFW Window", NULL, NULL);

    if (!window)
    {
        std::cerr << "Failed to open GLFW window\n";
        glfwTerminate();
        return nullptr;
    }

    glfwSetWindowUserPointer(window, &viewCamera); // это важно!

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, frambuffer_size_callback);
    glfwSetScrollCallback(window, mouseScrollCallbacl);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // ИНИЦИАЛИЗАЦИЯ GLAD - это важно!
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD\n";
        glfwTerminate();
        return nullptr;
    }

    glfwSwapInterval(0); // отключаем Vsync

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

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);


    ImGui_ImplOpenGL3_Init(glsl_version);

    return window;
}


int main() {

    GLFWwindow* window = init();

    if (!window)
    {
        std::cout << "Error init" << std::endl;
        return -1;
    }

    createFrameBuffer();
    // Устанавливаем viewport
    glViewport(0, 0, WIDTH, HEIGHT);

// настройка вершинных буферов
    unsigned int VAO, instanceVBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &instanceVBO); 

    glBindVertexArray(VAO);


    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, pointPosition.size() * sizeof(glm::vec3), pointPosition.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribDivisor(0, 1); 


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


        if(viGui::cloudOpen)
        {
            if(!initCloud)
            {
                glBindVertexArray(VAO);
                glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
                glBufferData(GL_ARRAY_BUFFER, pointPosition.size() * sizeof(glm::vec3), pointPosition.data(), GL_STATIC_DRAW);

                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
                glEnableVertexAttribArray(0);
                glVertexAttribDivisor(0, 1); 

                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
                initCloud = true;
            }
            // glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

            glViewport(0, 0, WIDTH, HEIGHT);

            // // Очищаем буферы
            // glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            // glClear(GL_COLOR_BUFFER_BIT);

            glBindVertexArray(VAO);
            
            view = viewCamera.moveCamera(window, deltaTime);
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
            glUniform4f(colorLoc,  point_color.x, point_color.y, point_color.z, point_color.w);
            glDrawArraysInstanced(GL_POINTS, 0, 1, viGui::cloud_size);
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

        viGui::ShowExampleAppMainMenuBar(pointPosition, intensity);


        if (viGui::show_BackroundColor)
        {
            ImGui::Begin("Background Color Render", &viGui::show_BackroundColor);
            
            ImGui::ColorEdit3("clear color", (float*)&clear_color);

            ImGui::End();
        }


        if(viGui::show_pointColor)
        {
            ImGui::Begin("Point Cloud COlor Render", &viGui::show_pointColor);
            
            ImGui::ColorEdit3("clear color", (float*)&point_color);

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
    }


    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);

    glfwTerminate();
    return 0;
}
