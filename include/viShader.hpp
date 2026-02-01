#ifndef VI_SHADER
#define VI_SHADER


#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unordered_map>
//#include <format>

namespace viShader {
    class Shader{
        public:
            Shader(const char* vertexPath, const char* fragmentPath);
            ~Shader() = default;

            void bind();
            void unbind();
            unsigned int ID; // ShaderProgramm ID

            bool setUniformCashe(const std::string& name);
            void setShaderMatrix4fv(const std::string& name, const glm::mat4& data);
            void setShader1i(const std::string& name, const int data);
            void setShaderMatrix4f(const std::string& name, const float data1,
                                   const float data2, const float data3, const float data4);
            
        private:

            const char* vShaderCode;
            const char* fShaderCode;

            unsigned int vertexShader, fragmentShader;

            void compileShader();
            void createProgram();
            int checkCompile(const unsigned int &shader);
            int checkProgram();

            std::unordered_map<std::string, uint> uniformCache;
    };
}

#endif