#ifndef VI_CAMERA
#define VI_CAMERA

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


namespace myCamera {
    class Camera{
        public:

            Camera()
            {
                cameraPos   = glm::vec3(0.0f, 0.0f,  60.0f);
                cameraFront = glm::vec3(0.0f, 0.f, -1.0f);
                cameraUp    = glm::vec3(0.0f, 1.0f,  0.0f);
                cameraRight = glm::vec3(1.0f, 0.0f, 0.0f);
                direction   = cameraFront;
                firstMouse  = true;
                yaw =  0.f;
                roll = 0.f;
                senseRoll = 1.f;

                currentOrient = glm::quat(1, 0, 0, 0);
            }

            Camera(glm::vec3 Pos, glm::vec3 Front, glm::vec3 Up) : cameraPos(Pos), 
                                                    cameraFront(Front), cameraUp(Up)
            {
               firstMouse = true; 
            }
    
            ~Camera() = default;

            glm::mat4 moveCamera(GLFWwindow *window, float deltaTime);
            void zoomCamera(double yoffset, float deltaTime);
            void processMouseMovement(double xpos, double ypos);
            void setWindowSize(int width, int height);
            void setXYPose(double xpos, double ypos);
            void setWorldModel(glm::mat4 *model, unsigned int modelLoc);
            void resetFirstMouse();
     static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
     static void mouseZoomCallback(double xpos, double ypos);

            void resetToZero();

        private:
            glm::vec3 cameraPos;        // положение камеры
            glm::vec3 cameraFront;      // вектор направления камеры (направление к цели)
            glm::vec3 cameraUp;         // вектор вверх
            glm::vec3 cameraRight;      // направление камеры в право
            glm::vec3 direction;

            glm::mat4 *model;            //ссылка на  матрицу мира
            unsigned int modelLoc;

            bool firstMouse;

            float yaw;
            float pitch;
            float roll;

            float senseRoll;
            

            int width;
            int height;

            float lastX;
            float lastY;

            glm::quat currentOrient;
    };
}


#endif