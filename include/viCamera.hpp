#ifndef VI_CAMERA
#define VI_CAMERA

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


namespace viCamera {
    enum class ProjectType : bool {Perspective , Ortho};

    struct CameraSettings {
        float fov = 75.0f;
        float nearPlanePerspectiv = 0.1f;
        float nearPlaneOrto = -1000.f;
        float farPlane = 1000.0f;
        bool usePerspective = true;

        struct Space {
            glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f,  60.0f);     // положение камеры
            glm::vec3 cameraFront = glm::vec3(0.0f, 0.f, -1.0f);     // вектор направления камеры (направление к цели)
            glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f,  0.0f);       // вектор вверх
            glm::vec3 cameraRight = glm::vec3(1.0f, 0.0f, 0.0f);     // направление камеры в право
        } cameraSpace;

        struct Speed {
            float scrool = 100.f;
            float move = 30.5f;
            float roll = 30.0f;
        } cameraSpeed;
    };

    class Camera{
        public:
            Camera(int width, int height , const CameraSettings& cameraSettings = {}) 
                                                    : _width(width) , _height(height),
                                                    _cameraSettings(cameraSettings),
                                                    _projectionType(ProjectType::Perspective)

            {
                model = glm::mat4(1.0f);
                currentOrient = glm::quat(1, 0, 0, 0);
                deltaTime = 0.0f;
                lastFrame = 0.0f;
                firstMouse  = true;
                scale = 0.1f;

                updateProjection();
            }

            ~Camera() = default;

            glm::mat4 moveCamera(GLFWwindow *window);
            void zoomCamera(double yoffset);
            void processMouseMovement(double xpos, double ypos);
            void setWindowSize(int width, int height);
            void setXYPose(double xpos, double ypos);
            void setWorldModel(unsigned int modelLoc, unsigned int projectionLoc);
            void resetFirstMouse();
            void resetToZero();
            void setOrthoProjection();
            void setPerspectiveProjection();
            void updateProjection();
            void toggleProjection();

            CameraSettings::Space& getCameraSpace() { return _cameraSettings.cameraSpace;}
            CameraSettings::Speed& getCameraSpeed() { return _cameraSettings.cameraSpeed;}

            glm::mat4 model;
            glm::mat4 projection;

//Call-back func
     static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
     static void mouseZoomCallback(double xpos, double ypos);
     static void mouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
     
        private:

            CameraSettings _cameraSettings;
            ProjectType _projectionType;

            unsigned int modelLoc;
            unsigned int projectionLoc;

            bool firstMouse;

            float yaw;
            float pitch;
            float roll;

            float senseRoll;
            

            int _width;
            int _height;

            float lastX;
            float lastY;

            glm::quat currentOrient;


            float deltaTime;	// время между текущим и последним кадрами
            float lastFrame; // время последнего кадра

            float scale; // увеличивайте этот коэффициент для увеличения
    };
}


#endif