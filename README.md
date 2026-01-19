# Virtuosa

OpenGL point cloud renderer.

## 🚀 Quick Start

### Clone repository
git clone --recursive https://github.com/TexA2/Virtuosa.git
cd Virtuosa

### Install dependencies (Ubuntu/Debian)
sudo apt-get install libglfw3-dev libglm-dev libpcl-dev libgtk-3-dev

### Build
mkdir build && cd build
cmake ..
make

### Run
./Virtuosa

## Camera Controls

**Rotation:**
- Mouse drag — Pitch/Yaw
- Q / E — Roll (rotation around Z-axis)

**Movement:**
- W / S — move along Z-axis
- A / D — move along X-axis
- Z / C — move along Y-axis

## Dependencies

- GLFW 3.3+
- GLM
- PCL (Point Cloud Library)
- GTK3 (optional)

## Note

This is a pilot version of the project. Functionality may change.

