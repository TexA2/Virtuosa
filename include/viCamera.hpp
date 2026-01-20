#ifndef VI_CAMERA
#define VI_CAMERA

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


namespace viCamera {
    enum class projectType : bool {Perspective , Ortho};

    struct typeCamera {
        float fov = 75.0f;
        float nearPlane = 0.1f;
        float farPlane = 1000.0f;
        bool usePerspective = true;

        glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f,  60.0f);     // положение камеры
        glm::vec3 cameraFront = glm::vec3(0.0f, 0.f, -1.0f);     // вектор направления камеры (направление к цели)
        glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f,  0.0f);       // вектор вверх
        glm::vec3 cameraRight = glm::vec3(1.0f, 0.0f, 0.0f);     // направление камеры в право
    };

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


                deltaTime = 0.0f;
                lastFrame = 0.0f;
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


            glm::mat4 model;
            glm::mat4 projection;
            bool projPerspective;

//Call-back func
     static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
     static void mouseZoomCallback(double xpos, double ypos);
     static void mouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
     
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


            float deltaTime;	// время между текущим и последним кадрами
            float lastFrame; // время последнего кадра

            float scale = 0.1f; // увеличивайте этот коэффициент для увеличения
    };
}


#endif