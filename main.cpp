#include <iostream>
#include <cmath>
#include <thread>
#include <chrono> 

#include <viWidget.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>


#define WIDTH  1280
#define HEIGHT 1024


unsigned int framebuffer;
unsigned int textureBuffer;

void createFrameBuffer() {
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);


    //создание текстуры рендеринга
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



int main() {

    viWidget::viMainWidget MainWindow;
    
    GLFWwindow* window = MainWindow.initMainWindow();

    if (!window)
    {
        std::cout << "Error init window" << std::endl;
        return -1;
    }

    MainWindow.initGui();
    MainWindow.initCamera();
    MainWindow.initShader();
    MainWindow.initCloudData();
    MainWindow.initUI();
    
    createFrameBuffer(); //В данный момент рудимент, но скоро нужно будет использовать

    MainWindow.getShader()->bind();

    unsigned int modelLoc = glGetUniformLocation(MainWindow.getShader()->ID, "model");
    unsigned int viewLoc = glGetUniformLocation(MainWindow.getShader()->ID, "view");
    unsigned int projectionLoc = glGetUniformLocation(MainWindow.getShader()->ID, "projection");

    
    unsigned int colorLoc = glGetUniformLocation(MainWindow.getShader()->ID, "ourColor");
    unsigned int useIntensityColorLoc = glGetUniformLocation(MainWindow.getShader()->ID, "useIntensityColor");

    MainWindow.getCamera()->setWorldModel(modelLoc, projectionLoc);   

    double lastTime = glfwGetTime();
    int frameCount = 0;

    MainWindow.getCamera()->resetToZero();

// ****Основной цикл****
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        glClearColor(MainWindow.getMenu()->clear_color.x,
                    MainWindow.getMenu()->clear_color.y,
                    MainWindow.getMenu()->clear_color.z,
                    MainWindow.getMenu()->clear_color.w); // Устанавливаем цвет очистки

        glClear(GL_COLOR_BUFFER_BIT); //| GL_DEPTH_BUFFER_BIT);

        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        if(MainWindow.getCloudData()->viCloud._cloud)
        {
            // glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

            glBindVertexArray(MainWindow.getCloudData()->viCloud.buffer.VAO);
            
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(MainWindow.getCamera()->moveCamera(window)));

            if (!MainWindow.getMenu()->show_intensity_color)
            {
                glUniform1i(useIntensityColorLoc, MainWindow.getMenu()->show_intensity_color);

                glVertexAttribDivisor(1, 0);           // Отключаем инстансинг
                glDisableVertexAttribArray(1);         // Отключаем атрибут

                glUniform4f(colorLoc,    MainWindow.getMenu()->point_color.x,
                            MainWindow.getMenu()->point_color.y,
                            MainWindow.getMenu()->point_color.z,
                            MainWindow.getMenu()->point_color.w);
            }
            else
            {
                glUniform1i(useIntensityColorLoc, MainWindow.getMenu()->show_intensity_color);

                glEnableVertexAttribArray(1);
                glVertexAttribDivisor(1, 1);
            }
            glDrawArraysInstanced(GL_POINTS, 0, 1,   MainWindow.getCloudData()->viCloud._cloud->size());
            glBindVertexArray(0);
            
            // glBindFramebuffer(GL_FRAMEBUFFER, 0);
            // glBindVertexArray(VAO);
            // glActiveTexture(GL_TEXTURE0);
            // glBindTexture(GL_TEXTURE_2D, textureBuffer); // Текстура из вашего фреймбуфера
            // glDrawArraysInstanced(GL_POINTS, 0, 1,cloud_size);
        }

// Start the Dear ImGui frame
        MainWindow.getMenu()->renderUI(window);


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

    glfwTerminate();
    return 0;
}
