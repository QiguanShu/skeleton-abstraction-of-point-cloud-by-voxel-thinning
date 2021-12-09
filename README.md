# skeleton abstraction of point cloud by voxel thinning

## Installation

### Prequisities

1. [pcl](https://pointclouds.org/downloads/)
2. [eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
3. [vtk](https://vtk.org/download/)
4. gdal
5. libodbc
6. grass

Currently, there is no way to automatically install those dependencies.
Install them as system dependencies instead. 

### Build

```shell
cd build
cmake -G 'Unix Makefiles' -DCMAKE_BUILD_TYPE=Debug  ..
make
```

Alternatively, open the `CMakeLists.txt` with your favorite IDE (VSCode, CLion, ...)
that has a CMake integration.

## Running

Documentation for this will come soon.
