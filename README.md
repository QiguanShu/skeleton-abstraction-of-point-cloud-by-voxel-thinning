# skeleton abstraction of point cloud by voxel thinning

## About
For detailed information about this research including methods and results: 
Middleton W, Shu Q, Ludwig F. Representing living architecture through skeleton reconstruction from point clouds. Sci Rep. 2022 Jan 28;12(1):1549. doi: 10.1038/s41598-022-05194-y. PMID: 35091577.
online available: https://www.nature.com/articles/s41598-022-05194-y
video demonstration of this program on Youtube: https://youtu.be/XHyCV2DjnSg

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
