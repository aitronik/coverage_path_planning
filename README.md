# coverage_path_planning

Internship Project

## Prerequisites

### Install CGAL dependencies

sudo apt install libcgal-dev

### Install CGAL 5.5 from source

- Download zip from the website (<https://github.com/CGAL/cgal/releases>)
- Extract files
- `mkdir build && cd build`
- `cmake ..`
- `sudo make install`
- `sudo ldconfig`

### Operations for debugging

rm -r build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
