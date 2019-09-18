### Build and Install PhysX
* Clone the latest PhysX git repository
* Run PhysX/physx/generate_projects.sh
* Go to PhysX/physx/compiler/linux-release 
* Run **make**, then **make install**

Note: Installation files will be in `PhysX/physx/install/linux`; Build files will be in `PhysX/physx/bin/linux.clang/release`

In Arch Linux, generate_projects.sh may fail. Try to add the following in physx/buildtools/presets/public/linux.xml
```
<cmakeParam name="CMAKE_LIBRARY_ARCHITECTURE" value="x86_64-linux-gnu" comment="library architecture" />
```
### Build

#### Prerequisite
* PhysX built and installed (see above section)
* CMake with version >= 3.15
* Standard c++ compiler supporting c++11 standard or above
* Dev libraries including `dl`,`pthread`,`GL`,`GLU`,`glut`,`X11`,`rt`,`cuda`
* `PythonLibs` with version >= 3.7.4 as well as PythonInterp (Should come with standard Python3 installation)
* `SWIG` with version >= 4.0.1

#### Build project
* Clone this repository
* Create a folder named build and cd to this folder
* Run cmake to the parent folder and specify the following arguments:
  * `PHYSX_INSTALL_DIR`
    * PhysX installation files folder 
    * e.g. physx/install/linux
  * `PHYSX_BUILD_DIR`
    * PhysX build files folder 
    * e.g. physx/bin/linux.clang/release
  * `EIGEN_DIR` 
    * eigen's source folder, can be directly cloned from their git repo
  * `BUILD_API`
    * Choose whether or not to build python API binding
    * Default ON
  * `BUILD_API_DIR`
    * Folder to output generated python files
    * Must be specified if BUILD_API is ON 
    * Default is ${CMAKE_BINARY_DIR}/pyPhysX)
  * `BUILD_EXAMPLES`
    * Choose whether or not to build c++ example, will deprecate soon
    * Default ON
  * `BUILD_TESTS` 
    * Choose whether or not to build unit tests
    * Default ON
* Run make

~~CMake Example:~~

```
cmake -DPHYSX_INSTALL_DIR='~/PhysX/physx/install/linux' -DPHYSX_BUILD_DIR='~/PhysX/physx/bin/linux.clang/release' -DEIGEN_DIR='~/eigen' ..
```
**cmake-gui** is recommended to configure this project. Hints about how to configure the required variables are already included in the variable descriptions

#### Generate Doxygen documents
Please install Doxygen first (http://www.doxygen.nl/download.html)
then run following command in repo directory (not build) to get documentations.
```
doxygen Doxyfile
```

### VSCode Setup
#### Plugins required
* C/C++
* CMake (Optional, to highlight cmake files)
* CMake Tools
* CMake Tools Helper
#### Configuration
##### Open File-/Preferences-/Settings-/Extensions
* Change Cmake Generator to Unix Makefiles
* Go to Cmake: Environment, open settings.json
* Add an item called "cmake.configureArgs", add put the following arguments into the array
  * "-DPHYSX_INSTALL_DIR='~/PhysX/physx/install/linux'"
  * "-DPHYSX_BUILD_DIR='~/PhysX/physx/bin/linux.clang/release'"
  * "-DEIGEN_DIR='~/eigen'"
* Reload the folder
