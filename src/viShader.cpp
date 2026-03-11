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

    bool Shader::setUniformCashe(const std::string& name) {

        if (uniformCache.find(name) == uniformCache.end())
        {
            uint location = glGetUniformLocation(ID, name.c_str());
            if (location != -1)
            {
                uniformCache[name] = location;
                std::cerr << "Uniform " << name << " ADD in shader cache!" << std::endl; 
                return true;
            }
            else
            {
                std::cerr << "Uniform " << name << " NOT found in shader!" << std::endl; 
                return false;
            }
        }
        return true;
    }

    void Shader::setShaderMatrix4fv(const std::string& name, const glm::mat4& data) {
        if(setUniformCashe(name))
            glUniformMatrix4fv(uniformCache[name], 1, GL_FALSE, glm::value_ptr(data));
    }

    void Shader::setShader1i(const std::string& name, const int data) {
        if(setUniformCashe(name))
            glUniform1i(uniformCache[name], data);
    }

    void Shader::setShaderMatrix4f(const std::string& name, const float data1,
                           const float data2, const float data3, const float data4) {
        if(setUniformCashe(name))
                glUniform4f(uniformCache[name], data1,
                            data2,
                            data3,
                            data4);
    }

    void Shader::computeTransform(const glm::mat4& transform, int pointCount) {
        GLuint computeShader;
        GLuint computeProgram;

        computeShader = glCreateShader(GL_COMPUTE_SHADER);

        std::ifstream fileStream("../shader/transformCloud.comp", std::ios::in);
        std::stringstream ShaderStream;
        ShaderStream << fileStream.rdbuf();
        fileStream.close();
        std::string Code = ShaderStream.str();

        const char* shaderSource = Code.c_str();
        glShaderSource(computeShader, 1, &shaderSource, NULL);
        glCompileShader(computeShader);

        GLint success;
        glGetShaderiv(computeShader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(computeShader, 512, NULL, infoLog);
            printf("Ошибка компиляции compute шейдера: %s\n", infoLog);
        }

        // Создание программы
        computeProgram = glCreateProgram();
        glAttachShader(computeProgram, computeShader);
        glLinkProgram(computeProgram);


        // Проверка на ошибки линковки
        glGetProgramiv(computeProgram, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(computeProgram, 512, NULL, infoLog);
            printf("Ошибка линковки программы: %s\n", infoLog);
        }

        glDeleteShader(computeShader);

        const GLuint LOCAL_SIZE = 256;
        GLuint numGroups = (pointCount + LOCAL_SIZE - 1) / LOCAL_SIZE;

        glUseProgram(computeProgram);

        //uniform transform !!! 
        glDispatchCompute(numGroups,1,1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    }