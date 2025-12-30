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
 bool show_BackroundColor = false;
 bool show_pointColor = false;


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


bool  cloudOpen = false;
bool  initCloud = false;
std::vector<float> intensity;  //(cloud_size * 3, 0.0f);
std::vector<glm::vec3> pointPosition; //(cloud_size);
int cloud_size;


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


void intensityToColor(float intensity, uint8_t& r, uint8_t& g, uint8_t& b) {

    // Нормализация интенсивности (предполагаем диапазон 0-1)
    intensity = std::max(0.0f, std::min(1.0f, intensity));
    
    // Градиент: синий (0) -> зеленый (0.5) -> красный (1)
    if (intensity < 0.5f) {
        // Синий -> Зеленый
        r = 0;
        g = static_cast<uint8_t>(2 * intensity * 255);
        b = static_cast<uint8_t>((1 - 2 * intensity) * 255);
    } else {
        // Зеленый -> Красный
        r = static_cast<uint8_t>((2 * intensity - 1) * 255);
        g = static_cast<uint8_t>((2 - 2 * intensity) * 255);
        b = 0;
    }
}

void pointCloudOpen(std::string path){
// откроем pcd файл
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud);

    cloud_size = cloud->width * cloud->height;
    intensity.resize(cloud_size * 3, 0.0f);
    pointPosition.resize(cloud_size);


// Запоминаем положение и интенсивность
    // std::vector<float> intensity (cloud_size * 3, 0.0f);

    float min_i = 0.f;
    float max_i = 0.f;


    // std::vector<glm::vec3> pointPosition(cloud_size);

    for (uint i = 0; i < cloud_size; ++i)
    {
        pointPosition[i] = glm::vec3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        min_i = std::min(min_i, cloud->points[i].intensity);
        max_i = std::max(max_i, cloud->points[i].intensity);
    }

    for (uint i = 0; i < cloud_size; ++i)
    {
        static uint pos = 0;
        float normalized_i = (cloud->points[i].intensity - min_i) / (max_i - min_i);  

        uint8_t r, g, b;
        intensityToColor(normalized_i, r, g, b);

        intensity[pos]   = r;
        intensity[++pos] = g;
        intensity[++pos] = b;

    }
    std::cout << "Razmer Cloud: "<< cloud_size << std::endl;
    cloudOpen = true;
}



static void ShowExampleMenuFile();

static void ShowExampleAppMainMenuBar()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            ShowExampleMenuFile();
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

static void ShowExampleMenuFile()
{
    IMGUI_DEMO_MARKER("Examples/Menu");
    if (ImGui::MenuItem("New")) {}
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
            pointCloudOpen(outPath);
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
    if (ImGui::MenuItem("Quit", "Alt+F4")) {}
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


        if(cloudOpen)
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
            glDrawArraysInstanced(GL_POINTS, 0, 1,cloud_size);
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

        ShowExampleAppMainMenuBar();


        if (show_BackroundColor)
        {
            ImGui::Begin("Background Color Render", &show_BackroundColor);
            
            ImGui::ColorEdit3("clear color", (float*)&clear_color);

            ImGui::End();
        }


        if(show_pointColor)
        {
            ImGui::Begin("Point Cloud COlor Render", &show_pointColor);
            
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
