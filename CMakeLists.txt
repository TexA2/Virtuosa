cmake_minimum_required(VERSION 3.10)
project(task4)

set(CMAKE_CXX_STANDARD 20)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)


include_directories(include)
include_directories(${GTK3_INCLUDE_DIRS})

# Добавляем библиотеку glad

add_library(glad STATIC "${CMAKE_CURRENT_SOURCE_DIR}/submodules/glad/src/glad.c")
target_include_directories(glad PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/submodules/glad/include")

# Добавляем библиотеку imgui
set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/submodules/imgui")
set(NFD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/submodules/nativefiledialog-extended/src")
set(STB_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/submodules/stb_image")
set(GLAD_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/submodules/glad/include")

# Создаем список исходных файлов для imgui
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

set(NFD_SOURCES
    ${NFD_DIR}/nfd_gtk.cpp)

# Создаем статическую библиотеку imgui
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC 
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)



add_library(nfd STATIC ${NFD_SOURCES})
target_include_directories(nfd PUBLIC 
    ${NFD_DIR}/include
)

# Настройка для OpenGL3 (используем glad вместо GLEW)
target_compile_definitions(imgui PRIVATE 
    IMGUI_IMPL_OPENGL_LOADER_GLAD
)

add_executable(main main.cpp src/viShader.cpp src/viCamera.cpp src/viWidget.cpp)

# Добавляем директории включаемых файлов
target_include_directories(main PRIVATE  
    ${GLAD_INCLUDE_PATH}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${NFD_DIR}/include
    ${GTK3_INCLUDE_DIRS}
    ${STB_IMAGE_PATH}
    ${glm_INCLUDE_DIRS}
)

# Связываем библиотеки
target_link_libraries(main 
    OpenGL::GL 
    glfw 
    glad 
    imgui
    nfd
    ${PCL_LIBRARIES}
    ${GTK3_LIBRARIES}
)
