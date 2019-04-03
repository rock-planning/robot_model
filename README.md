robot_model
=============
Robot Model is responsible for holding the information regarding the robot joints and links.
The robot model could also be used for checking collision detection.

# Requirements

![robotmodel](/uploads/97ab9ad5b73813fcf8b4bd2f9ebc2c29/robotmodel.png)

As shown in the above figure this package depends on collision detection and kinematics
libraries:
- [Kinematics Library](https://git.hb.dfki.de/dfki-planning/kinematics_library)
- [Collision Detection Library](https://git.hb.dfki.de/dfki-planning/collision_detection)


Installation
------------
On any Unix based operating systems, you can install this package. A simple way to install this package is to
use [Rock](https://www.rock-robotics.org/) framework. As this package is dependent only on the [base-types](https://github.com/rock-core/base-types)
of the [Rock](https://www.rock-robotics.org/) framework, it can also be build independent of the [Rock](https://www.rock-robotics.org/) framework.


Rock CMake Macros
-----------------

This package uses a set of CMake helper shipped as the Rock CMake macros.
Documentations is available on [this page](http://rock-robotics.org/stable/documentation/packages/cmake_macros.html).

Rock Standard Layout
--------------------

This directory structure follows some simple rules, to allow for generic build
processes and simplify reuse of this project. Following these rules ensures that
the Rock CMake macros automatically handle the project's build process and
install setup properly.

### Folder Structure

| directory         |       purpose                                                        |
| ----------------- | ------------------------------------------------------               |
| src/              | Contains all header (*.h/*.hpp) and source files                     |
| build/ *          | The target directory for the build process, temporary content        |
| bindings/         | Language bindings for this package, e.g. put into subfolders such as |
| ruby/             | Ruby language bindings                                               |
| viz/              | Source files for a vizkit plugin / widget related to this library    |
| resources/        | General resources such as images that are needed by the program      |
| configuration/    | Configuration files for running the program                          |
| external/         | When including software that needs a non standard installation process, or one that can be easily embedded include the external software directly here |
| doc/              | should contain the existing doxygen file: doxygen.conf               |
