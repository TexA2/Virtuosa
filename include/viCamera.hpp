#ifndef VI_CAMERA
#define VI_CAMERA

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


namespace myCamera {


    enum class projectType : bool {Perspective , Ortho};

    class Camera{
        public:

            Camera(int width, int height) : _width(width) , _height(height)
            {
                cameraPos   = glm::vec3(0.0f, 0.0f,  60.0f);
                cameraFront = glm::vec3(0.0f, 0.f, -1.0f);
                cameraUp    = glm::vec3(0.0f, 1.0f,  0.0f);
                cameraRight = glm::vec3(1.0f, 0.0f, 0.0f);

                model = glm::mat4(1.0f);

                direction   = cameraFront;
                firstMouse  = true;
                yaw =  0.f;
                roll = 0.f;
                senseRoll = 1.f;

                currentOrient = glm::quat(1, 0, 0, 0);

                //setOrthoProjection();
                setPerspectiveProjection();
                projPerspective = true;
            }

            ~Camera() = default;

            glm::mat4 moveCamera(GLFWwindow *window, float deltaTime);
            void zoomCamera(double yoffset, float deltaTime);
            void processMouseMovement(double xpos, double ypos);
            void setWindowSize(int width, int height);
            void setXYPose(double xpos, double ypos);
            void setWorldModel(unsigned int modelLoc, unsigned int projectionLoc);
            void resetFirstMouse();
     static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
     static void mouseZoomCallback(double xpos, double ypos);

            void resetToZero();
            void setOrthoProjection();
            void setPerspectiveProjection();


            glm::mat4 model;
            glm::mat4 projection;
            bool projPerspective;
        private:
            glm::vec3 cameraPos;        // положение камеры
            glm::vec3 cameraFront;      // вектор направления камеры (направление к цели)
            glm::vec3 cameraUp;         // вектор вверх
            glm::vec3 cameraRight;      // направление камеры в право
            glm::vec3 direction;

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
    };
}


#endif