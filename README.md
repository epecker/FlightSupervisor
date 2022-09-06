# CVLAD Supervisory Controller

## Contents

<!-- TOC -->
* [CVLAD Supervisory Controller](#cvlad-supervisory-controller)
  * [Contents](#contents)
  * [About The Project](#about-the-project)
  * [Built With](#built-with)
  * [Prerequisites](#prerequisites)
    * [GNU/Linux - Ubuntu](#gnulinux---ubuntu)
    * [MacOS - XCode with Homebrew](#macos---xcode-with-homebrew)
    * [Windows 10 - Visual Studio Community](#windows-10---visual-studio-community)
  * [Build Guide](#build-guide)
    * [GNU/Linux - Ubuntu](#gnulinux---ubuntu)
    * [MacOS - XCode with Homebrew](#macos---xcode-with-homebrew)
    * [Windows 10 - Visual Studio Community](#windows-10---visual-studio-community)
  * [Contact](#contact)
  * [Acknowledgements](#acknowledgements)
<!-- TOC -->

## About The Project

Supervisory Controller modelled using DEVS that can be deployed onto the Bell-412 as part of the
closed-loop autonomy systems used by CVLAD.

## Built With

* Boost<span>.</span>org
* CMake
* SimulationEverywhere/Cadmium
* SimulationEverywhere/NDTime

## Prerequisites

Note: For CMake to automatically find boost, the version of Boost used must be listed in the variable 
_Boost_KNOWN_VERSIONS in CMake's FindBoost.cmake file. For example, CMake 3.24.1 supports up to Boost version 1.79.0.
For CMake 3.24.1, this information can be found at [https://github.com/Kitware/CMake/blob/v3.24.1/Modules/FindBoost.cmake](https://github.com/Kitware/CMake/blob/v3.24.1/Modules/FindBoost.cmake).
Where v3.24.1 can be swapped to your version of cmake.

### GNU/Linux - Ubuntu

Using a terminal perform the following.

* Update the Ubuntu repositories

	```bash
	apt update
	```

* Upgrade the Ubuntu System

	```bash
	apt upgrade
	```

* Install the ubuntu development tools

	```bash
	apt install build-essential
	```

* Install Boost

	```bash
	apt install libboost-all-dev
	```

* Install CMake via snap

	```bash
	snap install cmake –classic
	```

* Install Git

	```bash
	apt install git
	```

### MacOS - XCode with Homebrew

Using a terminal perform the following.

* Install cmake
	```bash
	brew install cmake
	```

* Install boost
	```bash
	brew install boost
	```

### Windows 10 - Visual Studio Community

Managing Visual Studio Community

* Install "Desktop development with C++" with the default options listed under workloads.

Installing Boost

* Navigate to https://www.boost.org/users/download/
* Navigate down to "Other Downloads"
* Click on "Prebuilt windows binaries"
* Navigate down to "1.75.0"
* Click on "1.75.0"
* Download a version of the boost binary.
	* msvc = Microsoft Visual C++
	* 14.2 = Version of the msvc compiler
		* 14.3 corresponds to Visual Community 2022.
		* 14.2 corresponds to Visual Community 2019.
		* 14.1 corresponds to Visual Community 2017.
		* Must select a version equal to or less than your current version of visual studio. For example, Visual Community 2022 uses msvc 14.3, but is compatable with the boost binaries for msvc 14.2 and msvc 14.1.
	* 32 = 32 bit binary
	* 64 = 64 bit binary
	* We are currently using "boost_1_75_0-msvc-14.2-64.exe"
* Install the boost library using the executable you downloaded above. The path **must** be set to the default location.

## Build Guide

Using the terminal perform the following.

* Navigate your repository folder

	```bash
	cd /path/to/repositories
	```

* Clone the FlightSupervisor repository using one of the following methods
	* https: `https://github.com/SimulationEverywhere-Models/FlightSupervisor.git`

		```bash
		git clone https://github.com/SimulationEverywhere-Models/FlightSupervisor.git
		```

	* ssh: `git@github.com:SimulationEverywhere-Models/FlightSupervisor.git`

		```bash
		git clone git@github.com:SimulationEverywhere-Models/FlightSupervisor.git
		```

* Continue with the platform specific instructions below

### GNU/Linux - Ubuntu

Using the terminal perform the following.

* Navigate to the repository

	```bash
	cd ./FlightSupervisor
	```

* Execute the setup script

	```bash
	./setup.sh -ibm
	```
   * i - initialize the submodules
   * b - build the submodules
   * m - Generate the build files

* Navigate to the build files

	```bash
	./build/make
	```
 
* Build the supervisor

  ```bash
  make supervisor
  ```

* You can now run the supervisor executable
* If you make any changes to the **CMakeLists.txt** you have two options to regenerate the make files
  1. Run `cmake ../..` while within the make folder
  2. Run `./setup.sh -m` while within the FlightSupervisor folder

### MacOS - XCode with Homebrew

Using the terminal perform the following.

* Navigate to the repository

  ```bash
  cd ./FlightSupervisor
  ```

* Reset the Xcode tool paths
  ```bash
  xcode-select --reset
  ```

* Execute the setup script

  ```bash
  ./setup.sh -ibx
  ```
	* i - initialize the submodules
	* b - build the submodules
	* x - Generate the Xcode solution files

* Open the Xcode solution file using Xcode
  * The solution is located in 

    ```bash
    ./build/xcode
    ```

* You can now build and run the project using Xcode
* If you make any changes to the **CMakeLists.txt** you have two options to regenerate the Xcode solution
  1. Run `cmake -G Xcode ../..` from within the xcode folder
  2. Run `./setup.sh -m` while within the FlightSupervisor folder

### Windows 10 - Visual Studio Community

* Open Visual Studio Community
* Click "Open a local folder"
* Open the repository
* Generate the build files
	* Visual Studio Community does this automatically
	* If it doesn't
		```
		Project > Configure Cache
		```
* Build the project
	```
	build > build all
	```
* Select a startup item to run the project
* Visual Studio Community will regenerate the cmake cache on **CMakeList.txt** changes.

## Contact

James Horner - JamesHorner@cmail.carleton.ca or jwehorner@gmail.com

Tanner Trautrim - trautrim.tbt@outlook.com

Project Link: [https://github.com/SimulationEverywhere-Models/FlightSupervisor](https://github.com/SimulationEverywhere-Models/FlightSupervisor)

## Acknowledgements
