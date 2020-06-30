# Using CMSIS DSP libaries in STM32CubeIDE v1.2.0
Follow these steps to initialize an STM32CubeIDE project with CMSIS DSP library header files and library links.

## 1. (One-time) Install CMSIS DSP package

* Open the CubeMX .ioc file
* Go to the Additional Software tab
* Select CMSIS in the Software Component Class
* In the Packs window select `CMSIS CORE` and `CMSIS DSP (Library)` for installation

<img src="Install_CMSIS_DSP_package.jpg"/>

## 2. Activate CMSIS DSP for your project

<img src="Activate_CMSIS_DSP_for_project.jpg"/>

## 3. Add CMSIS DSP header files to compiler include path

<img src="Specify_CMSIS_DSP_include_path.jpg"/>

## 4. Specify Linker path and name for CMSIS DSP library

<img src="Specify_CMSIS_DSP_linker_library_name_path.jpg"/>


