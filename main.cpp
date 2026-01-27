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
    //createFrameBuffer(); //В данный момент рудимент, но скоро нужно будет использовать

// ****Основной цикл****
    while (!glfwWindowShouldClose(MainWindow.getWindow()))
        MainWindow.render();

    return 0;
}
