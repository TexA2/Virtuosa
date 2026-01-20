#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <viCamera.hpp>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>


using namespace viCamera;

    void Camera::zoomCamera(double yoffset){
        float zoomSpeed = 150.0f * deltaTime;

        getCameraSpace().cameraPos += getCameraSpace().cameraFront * (float)yoffset * zoomSpeed;
    }

    glm::mat4 Camera::moveCamera(GLFWwindow *window){

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;  

        float cameraSpeed = 30.5f * deltaTime;
        float rollSpeed = 90.0f * deltaTime; // Скорость вращения в градусах в секунду

    // тут двигаемся по Z
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            {
                getCameraSpace().cameraPos +=  getCameraSpace().cameraFront * cameraSpeed;
                scale -= 0.1 * deltaTime;
            }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            {
                getCameraSpace().cameraPos -= getCameraSpace().cameraFront * cameraSpeed;
                scale += 0.1 * deltaTime;
            }

    // тут двигаемся по Х
    glm::vec3 right = glm::normalize(glm::cross(getCameraSpace().cameraFront, getCameraSpace().cameraUp));
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            getCameraSpace().cameraPos -= right * cameraSpeed;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            getCameraSpace().cameraPos += right * cameraSpeed;


        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
            getCameraSpace().cameraPos += getCameraSpace().cameraUp * cameraSpeed;
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
            getCameraSpace().cameraPos -= getCameraSpace().cameraUp * cameraSpeed;


        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        {
            float rollSpeed = 30.0f * deltaTime;
            
            // Roll - вращение вокруг локальной оси Z (направления взгляда)
            // В системе камеры: ось Z = cameraFront (направление взгляда)
            glm::quat rollRotation = glm::angleAxis(glm::radians(rollSpeed), getCameraSpace().cameraFront);
            
            currentOrient = rollRotation * currentOrient;
            currentOrient = glm::normalize(currentOrient);
            
            getCameraSpace().cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
        }

            
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        {
            float rollSpeed = -30.0f * deltaTime; 
            
            glm::quat rollRotation = glm::angleAxis(glm::radians(rollSpeed), getCameraSpace().cameraFront);
            
            currentOrient = rollRotation * currentOrient;
            currentOrient = glm::normalize(currentOrient);
            
            getCameraSpace().cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
        }


        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
   
        return glm::lookAt(getCameraSpace().cameraPos,
                            getCameraSpace().cameraPos + getCameraSpace().cameraFront,
                            getCameraSpace().cameraUp);
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
    getCameraSpace().cameraFront = currentOrient * glm::vec3(0.0f, 0.0f, -1.0f);
    
    // Также обновляем up-вектор для сохранения ортогональности
    getCameraSpace().cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
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
        this->_width  = width;
        this->_height = height;
    }

    void Camera::setXYPose(double xpos, double ypos){
        this->lastX = xpos;
        this->lastY = ypos;
    }

    void Camera::setWorldModel(unsigned int modelLoc, unsigned int projectionLoc)
    {
        this->modelLoc = modelLoc;
        this->projectionLoc = projectionLoc;
    }

    void Camera::resetFirstMouse() { firstMouse = true; }

    void Camera::resetToZero() {
        getCameraSpace() = CameraSettings::Space();
        
        this->firstMouse  = true;
        this->currentOrient = glm::quat(1, 0, 0, 0);
    }

    void Camera::setOrthoProjection() {
            projection= glm::ortho(
                (-float(_width)  / 2.0f) * scale,    // left
                (float(_width)   / 2.0f) * scale,    // right
                (-float(_height) / 2.0f) * scale,    // bottom
                (float(_height)  / 2.0f) * scale,    // top
                -1000.f,                     // near
                1000.0f                   // far
            );
    } 

    void Camera::setPerspectiveProjection() {
        projection = glm::perspective(glm::radians(75.f),
                                      float(_width) / float(_height),
                                      0.1f, 1000.f);
    } 


    void Camera::mouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
       
        Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
        
        if (camera)
        {
            float currentFrame = glfwGetTime();
            camera->deltaTime = currentFrame - camera->lastFrame;
            camera->lastFrame = currentFrame;  

            camera->zoomCamera(yoffset);
        }
    }    