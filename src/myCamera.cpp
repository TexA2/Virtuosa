#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <myCamera.hpp>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>


using namespace myCamera;


    void Camera::zoomCamera(double yoffset, float deltaTime){
        float zoomSpeed = 150.0f * deltaTime;

        cameraPos += cameraFront * (float)yoffset * zoomSpeed;
    }

    glm::mat4 Camera::moveCamera(GLFWwindow *window, float deltaTime){

        float cameraSpeed = 30.5f * deltaTime;
        float rollSpeed = 90.0f * deltaTime; // Скорость вращения в градусах в секунду

    // тут двигаемся по Z
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            cameraPos +=  cameraFront * cameraSpeed;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            cameraPos -= cameraFront * cameraSpeed;

    // тут двигаемся по Х
    glm::vec3 right = glm::normalize(glm::cross(cameraFront, cameraUp));
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            cameraPos -= right * cameraSpeed;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            cameraPos += right * cameraSpeed;


        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
            cameraPos += cameraUp * cameraSpeed;
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
            cameraPos -= cameraUp * cameraSpeed;


        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        {
            float rollSpeed = 30.0f * deltaTime;
            
            // Roll - вращение вокруг локальной оси Z (направления взгляда)
            // В системе камеры: ось Z = cameraFront (направление взгляда)
            glm::quat rollRotation = glm::angleAxis(glm::radians(rollSpeed), cameraFront);
            
            currentOrient = rollRotation * currentOrient;
            currentOrient = glm::normalize(currentOrient);
            
            cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
        }

            
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        {
            float rollSpeed = -30.0f * deltaTime; 
            
            glm::quat rollRotation = glm::angleAxis(glm::radians(rollSpeed), cameraFront);
            
            currentOrient = rollRotation * currentOrient;
            currentOrient = glm::normalize(currentOrient);
            
            cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
        }


        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(*model));
        return glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
    }

    void Camera::processMouseMovement(double xpos, double ypos){

     // Инициализация при первом вызове
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
        return;
    }

    // Вычисляем смещение от предыдущей позиции
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // Обратный порядок, так как y-координаты идут сверху вниз
    
    // Обновляем последние позиции
    lastX = xpos;
    lastY = ypos;
    
    float sensitivity = 0.001f; // Уменьшено для работы с радианами
    
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    // ВАРИАНТ 1: Самый правильный - накопление вращений через кватернионы
    // (предполагаем, что у вас есть переменная для хранения вращения объекта)
    
    // Вращение вокруг глобальной оси Y (горизонтальное движение мыши)
    glm::quat yawRotation = glm::angleAxis(xoffset, glm::vec3(0.0f, 1.0f, 0.0f));
    
    // Вращение вокруг локальной оси X (вертикальное движение мыши)
    // Получаем локальную ось X из текущего вращения объекта
    glm::vec3 localRight = currentOrient * glm::vec3(1.0f, 0.0f, 0.0f);
    glm::quat pitchRotation = glm::angleAxis(yoffset, localRight);
    
    // Комбинируем вращения: pitch * orientation * yaw
    currentOrient = pitchRotation * currentOrient * yawRotation;
    currentOrient = glm::normalize(currentOrient);
    
    // Обновляем направление камеры (если нужно)
    cameraFront = currentOrient * glm::vec3(0.0f, 0.0f, -1.0f);
    
    // Также обновляем up-вектор для сохранения ортогональности
    cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
    }

    void Camera::mouseMoveCallback(GLFWwindow* window, double xpos, double ypos){
        
        void* ptr = glfwGetWindowUserPointer(window);

        if (ptr)
        {
            Camera* camera = static_cast<Camera*>(ptr);
            camera->processMouseMovement(xpos, ypos);

            int width, height;
            glfwGetWindowSize(window, &width, &height);
            camera->setWindowSize(width, height);
            camera->setXYPose(xpos, ypos);
        }
    }

    void Camera::setWindowSize(int width, int height){
        this->width  = width;
        this->height = height;
    }

    void Camera::setXYPose(double xpos, double ypos){
        this->lastX = xpos;
        this->lastY = ypos;
    }

    void Camera::setWorldModel(glm::mat4 *model, unsigned int modelLoc)
    {
        this->model = model;
        this->modelLoc = modelLoc;
    }

    void Camera::resetFirstMouse() { firstMouse = true; }

    void Camera::resetToZero() {
        this->cameraPos   = glm::vec3(0.0f, 0.0f,  60.0f);
        this->cameraFront = glm::vec3(0.0f, 0.f, -1.0f);
        this->cameraUp    = glm::vec3(0.0f, 1.0f,  0.0f);
        this->cameraRight = glm::vec3(1.0f, 0.0f, 0.0f);
        this->direction   = cameraFront;
        this->firstMouse  = true;
        this->yaw =  0.f;
        this->roll = 0.f;
        this->senseRoll = 1.f;

        this->currentOrient = glm::quat(1, 0, 0, 0);
    }