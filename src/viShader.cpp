#include <viShader.hpp>

using namespace viShader;

    Shader::Shader(const char* vertexPath, const char* fragmentPath) {
    
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;

        vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);


        try
        {
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);

            std::stringstream vShaderStream, fShaderStream;

            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf(); 

            vShaderFile.close();
            fShaderFile.close();

            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();  
        }
        catch(std::ifstream::failure e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }

        vShaderCode = vertexCode.c_str();
        fShaderCode = fragmentCode.c_str();

        compileShader();
        createProgram();
    }

    void Shader::compileShader() {

        vertexShader = glCreateShader(GL_VERTEX_SHADER);
        fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

        glShaderSource(vertexShader, 1, &vShaderCode, NULL);
        glShaderSource(fragmentShader, 1, &fShaderCode, NULL);

        glCompileShader(vertexShader);
        glCompileShader(fragmentShader);

        checkCompile(vertexShader);
        checkCompile(fragmentShader);
    }

    int Shader::checkCompile(const unsigned int &shader) {
 
        int result;
        char infoLog[512];
        glGetShaderiv(shader, GL_COMPILE_STATUS, &result);

        if(!result)
        {
            glGetShaderInfoLog(shader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
            return result;
        }

        return result;
    }

    void Shader::createProgram(){
        ID = glCreateProgram();
        glAttachShader(ID, vertexShader);
        glAttachShader(ID, fragmentShader);
        glLinkProgram(ID);

        checkProgram();

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }


    int Shader::checkProgram() {

        int result;
        char infoLog[512];

        glGetProgramiv(ID, GL_LINK_STATUS, &result);

        if(!result)
        {
            glGetProgramInfoLog(ID, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER_PROGRAN::COMPILATION_FAILED\n" << infoLog << std::endl;
            return result;
        }

        return result;
    }

    void Shader::bind() {
         glUseProgram(ID);
    }

    void Shader::unbind() {
        glUseProgram(0);
    }