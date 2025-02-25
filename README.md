# True Turn-Radius Constrained Shortest Paths - Implementation


## Getting Started
### Dependencies
1. C++ Compiler with C++23 Support
	* GCC 13+ (`g++ --version`)
	* Clang 16+ (`clang++ --version`)
	* MSVC (Visual Studio 2022 17.6+)
2. CMake 3.26+
   * CMake is required for building this project. Install it from:
      * CMake Official Site
	  * Package Managers
	      * macOS: `brew install cmake`
		  * Ubuntu `sudo apt install cmake`
		  * Windows: Use the installer or `choco install cmake` (Chocolatey)

### Installation and Build
1. **Install Dependencies**
   * C++ Compiler with C++23 support & CMake 3.26+
2. Clone the Repository
3. Build the Project
	* **On Linux/macOS:**
	```bash
	mkdir build
	cd build
	cmake ..
	cmake --build .
	```
	* **On Windows (with CMake and MSVC):**
	```bash
	mkdir build && cd build
	cmake .. -G "Visual Studio 17 2022"
	cmake --build .
	```
4. Run the Executable
   ```sh
	./planar 	# Linux/macOS
	./planar.exe # Windows
	``` 

## Graph Generation Parameters

## Modifying Obstalce Generation


