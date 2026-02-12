#include <viCamera.hpp>
#include <iostream>
#include <viWidget.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>


using namespace viCamera;

    void Camera::zoomCamera(double yoffset){
        float zoomSpeed = getCameraSpeed().scrool * deltaTime;

        getCameraSpace().cameraPos += getCameraSpace().cameraFront * (float)yoffset * zoomSpeed;
        scale -= 0.1 * yoffset * zoomSpeed;

    }

    // TODO: Камера не должна перехватывать window
    // пусть Widget перехватывает window и от события вызывает действие
    // но для начала нужно синхронизировать проекции!
    glm::mat4 Camera::moveCamera(GLFWwindow *window) {

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;  

        float cameraSpeed = getCameraSpeed().move * deltaTime;

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
            float rollSpeed = getCameraSpeed().roll * deltaTime;
            
            // Roll - вращение вокруг локальной оси Z (направления взгляда)
            // В системе камеры: ось Z = cameraFront (направление взгляда)
            glm::quat rollRotation = glm::angleAxis(glm::radians(rollSpeed), getCameraSpace().cameraFront);
            
            currentOrient = rollRotation * currentOrient;
            currentOrient = glm::normalize(currentOrient);
            
            getCameraSpace().cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
        }

            
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        {
            float rollSpeed = -getCameraSpeed().roll * deltaTime; 
            
            glm::quat rollRotation = glm::angleAxis(glm::radians(rollSpeed), getCameraSpace().cameraFront);
            
            currentOrient = rollRotation * currentOrient;
            currentOrient = glm::normalize(currentOrient);
            
            getCameraSpace().cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
        }

        updateProjection();
        lookAt = glm::lookAt(getCameraSpace().cameraPos,
                             getCameraSpace().cameraPos + getCameraSpace().cameraFront,
                             getCameraSpace().cameraUp);

        return getMvpMatrix();
    }

    void Camera::processMouseMovement(double xpos, double ypos) {

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


    glm::quat yawRotation = glm::angleAxis(xoffset, glm::vec3(0.0f, 1.0f, 0.0f));
    
    glm::vec3 localRight = currentOrient * glm::vec3(1.0f, 0.0f, 0.0f);
    glm::quat pitchRotation = glm::angleAxis(yoffset, localRight);
    
    // Комбинируем вращения: pitch * orientation * yaw
    currentOrient = pitchRotation * currentOrient * yawRotation;
    currentOrient = glm::normalize(currentOrient);
    

    getCameraSpace().cameraFront = currentOrient * glm::vec3(0.0f, 0.0f, -1.0f);
    getCameraSpace().cameraUp = currentOrient * glm::vec3(0.0f, 1.0f, 0.0f);
    }

    void Camera::mouseMoveCallback(GLFWwindow* window, double xpos, double ypos){
        
        void* ptr = glfwGetWindowUserPointer(window);

        if (ptr)
        {
            viWidget::viMainWidget* widget = static_cast<viWidget::viMainWidget*>(ptr);
            widget->getCamera()->processMouseMovement(xpos, ypos);

            int width, height;
            glfwGetWindowSize(window, &width, &height);

            widget->getCamera()->setXYPose(xpos, ypos);
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

    void Camera::resetFirstMouse() { firstMouse = true; }

    void Camera::resetToZero() {
        getCameraSpace() = CameraSettings::Space();
        
        resetFirstMouse();
        this->currentOrient = glm::quat(1, 0, 0, 0);
    }

    void Camera::setOrthoProjection() {
            projection= glm::ortho(
                (-float(_width)  / 2.0f) * scale,    // left
                (float(_width)   / 2.0f) * scale,    // right
                (-float(_height) / 2.0f) * scale,    // bottom
                (float(_height)  / 2.0f) * scale,    // top
                _cameraSettings.nearPlaneOrto,       // near
                _cameraSettings.farPlane             // far
            );
    } 

    void Camera::setPerspectiveProjection() {
        projection = glm::perspective(glm::radians(75.f),
                                      float(_width) / float(_height),
                                      _cameraSettings.nearPlanePerspectiv,
                                      _cameraSettings.farPlane);
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
    
    
    void Camera::updateProjection() {
        if (_projectionType == ProjectType::Perspective)
            setPerspectiveProjection();
        else 
            setOrthoProjection();
    }

    void Camera::toggleProjection() {

        if (_projectionType == ProjectType::Perspective)
            _projectionType = ProjectType::Ortho;
        else
            _projectionType = ProjectType::Perspective;

        updateProjection();
    }

    glm::mat4 Camera::getMvpMatrix() {
        return model * projection * lookAt;
    }

    void Camera::rayCast(GLFWwindow* window, double xpos, double ypos) {

        void* ptr = glfwGetWindowUserPointer(window);
        double adjustedX = 0;

        if (ptr)
        {
            viWidget::viMainWidget* widget = static_cast<viWidget::viMainWidget*>(ptr);
            adjustedX = xpos - (_width * widget->getObjectPanelWidth());
        }

        float ndcX  = (2.f * adjustedX) /  _width - 1.0f;
        float ndcY  = 1.f - (2.f * ypos) / _height;

        std::cout << "Screen: (" << xpos << ", " << ypos << ")" << std::endl;
        std::cout << "NDC: (" << ndcX << ", " << ndcY << ")" << std::endl;
        std::cout << "Render area size: " << _width << " x " << _height << std::endl;

        // NDC → Clip Space
        glm::vec4 rayClip(ndcX, ndcY, -1.0f, 1.0f);

        // Clip Space → Eye Space (View Space)
        glm::vec4 rayEye = glm::inverse(projection) * rayClip;
        rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f);

        // Eye Space → World Space
        glm::vec3 rayWorld = glm::vec3(glm::inverse(lookAt) * rayEye);
        rayWorld = glm::normalize(rayWorld);

        glm::vec3 rayOrigin = _cameraSettings.cameraSpace.cameraPos;

        // 7. Теперь можно выполнять тесты на пересечение с облаками точек
        std::cout << "Ray Origin: (" << rayOrigin.x << ", " << rayOrigin.y << ", " << rayOrigin.z << ")" << std::endl;
        std::cout << "Ray Direction: (" << rayWorld.x << ", " << rayWorld.y << ", " << rayWorld.z << ")" << std::endl;

        std::cout << std::endl;
    }