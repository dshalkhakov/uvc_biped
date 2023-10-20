# Upper Body Vertical Control Code

This repo hosts simulator and controller code for biped robots. The controller implements [Upper Body Vertical Control method](https://ai2001.ifdef.jp/uvc/uvc_Eng.html).

# Motivation

I've stumbled upon UVC method while researching bipedal robots and found it very promising. This codebase
is intended for further experimentation on this method and applying it to real robots.

# Getting Started

Prerequisites:

1. At least Visual Studio 2022 v17.2.4 64-bit with C/C++ workloads
2. [vcpkg](https://vcpkg.io/en/getting-started) cloned, built and `VCPKG_ROOT` env variable set

Building `src` project:

1. Open `src` folder in Visual Studio.
2. Everything should build, provided `vcpkg` and `cmake` are available.
3. HW interactions have been stubbed out, so it builds but won't really do anything. Please run and build the tests projects to see the code in action.

Building `sim` project:

1. `ode-0.16.3` should be downloaded and built separately. Basically same workflow should work: open folder in Visual Studio, and build.
2. ODE `drawstuff` library textures should be put under `C:\ode-0.13\drawstuff\textures` (the path is hardcoded, sorry)
2. Open `sim` folder in Visual Studio. `ode` dependency paths are hardcoded in `sim/CMakeLists.txt`, so if you're building under something different that x64-windows,
   things will most likely need manual intervention.
3. Build and run.

# Credits

Original author of the code is Dr.Guero. Code was imported from http://ai2001.ifdef.jp/ at 10-16-2023.
Translation from Japanese and further modifications were made by yours truly.

# License

Please refer to original license at http://ai2001.ifdef.jp/.
