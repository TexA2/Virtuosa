#ifndef MY_SHADER
#define MY_SHADER


#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
//#include <format>

namespace myShader {
    class Shader{
        public:
            Shader(const char* vertexPath, const char* fragmentPath);
            ~Shader() = default;

            void use();
            unsigned int ID; // ShaderProgramm ID
            
        private:

            const char* vShaderCode;
            const char* fShaderCode;

            unsigned int vertexShader, fragmentShader;

            void compileShader();
            void createProgram();
            int checkCompile(const unsigned int &shader);
            int checkProgram();
    };
}

#endif