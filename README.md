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

### Authorization

To be able to fully utilise the Supervisor, access to the following dependancies is required:
* The [CVLAD Shared Memory Model](https://bitbucket.org/frl-student/sharedmemorymodel)
* The [Reliable UDP Library](https://github.com/jwehorner/RUDP) 

RUDP is a publicly accessible repository, so no authorization is required to access it. The shared memory model however is intellectual property of the NRC and requires authorization. To gain access to the repository and make it available for initialisation:
1. Create a Bitbucket account.
2. Ask an admin of the Shared Memory Model repo to add you as a user. To do so contact James at jameshorner@cmail.carleton.ca and send your bitbucket email.
3. Set up an SSH key on your machine and add the public key to your Bitbucket, following [this guide](https://support.atlassian.com/bitbucket-cloud/docs/set-up-an-ssh-key/).
4. Continue on with the cloning of the repository.

### CMake FindBoost Compatibility

Note: For CMake to automatically find boost, the version of Boost used must be listed in the variable 
_Boost_KNOWN_VERSIONS in CMake's FindBoost.cmake file. For example, CMake 3.24.1 supports up to Boost version 1.79.0.
For CMake 3.24.1, this information can be found at [https://github.com/Kitware/CMake/blob/v3.24.1/Modules/FindBoost.cmake](https://github.com/Kitware/CMake/blob/v3.24.1/Modules/FindBoost.cmake).
Where v3.24.1 can be swapped to your version of cmake.

### GNU/Linux - Ubuntu

Using a terminal perform the following.

* Update the Ubuntu repositories

	```bash
	sudo apt update
	```

* Upgrade the Ubuntu System

	```bash
	sudo apt upgrade
	```

* Install the ubuntu development tools

	```bash
	sudo apt install build-essential
	```

* Install Boost

	```bash
	sudo apt install libboost-all-dev
	```

* Install CMake
	* Download the precompiled binary with the SH extension from the [CMake Download Page](https://cmake.org/download/)
	* Allow the script to be executable (either using the terminal or in files)
		* In the Downloads folder of Files, right click on the script and select "Properties", then "Permissions" and enable the "Execute" checkbox.
	* Run the script in a terminal using the ```--skip-license``` and ```--prefix=/usr/local``` (change the prefix to whatever your desired installation directory is).
	```bash
	sudo ./cmake.sh --skip-license --prefix=/usr/local
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

### Windows 10 / 11 - Visual Studio Community

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

### Windows 10 / 11 - Visual Studio Community

To build the models there are two alternatives:
1) Use Visual Studio Community
2) Use a PowerShell script.

#### Visual Studio Community

* Initialise and update the git submodules:

	```powershell
	git submodule init
	git submodule update --remote --recursive
	```
* Open Visual Studio Community
* Click "Open a local folder"
* Open the module RUDP
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
* The static library `rudp.lib` should have been created in the directory `deps\RUDP\build\Debug\`.

**Disclaimer**: if you got the following error: 
```
Error LNK1104: cannot open file 'libboost_regex-vc142-mt-gd-x64-1_75.lib
``` 
then edit the **CMakeLists.txt** to find the `regex` component for boost, as follows:

```
find_package(
	Boost 1.65 REQUIRED
	COMPONENTS system thread regex
	)
```
* Then, still in Visual Studio Community click "Open a local folder"
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

#### PowerShell script

Using the PowerShell terminal perform the following. 

* Make sure that scripts can be run as a standard user, otherwise run PowerShell as Administrator and type: 
```PowerShell
Set-ExecutionPolicy RemoteSigned
```
* Navigate to the repository

	```PowerShell
	cd .\FlightSupervisor
	```

* Execute the setup script

	```PowerShell
	.\setup.ps1 -i -b -m
	```
   * i - initialize the submodules
   * b - build the submodules
   * m - build the supervisor model and test drivers

* You can now run the supervisor executable. You will find it under `build\make\src\Debug\`.

* If you make any changes to the **CMakeLists.txt** you have two options to regenerate the make files
  1. Run `cmake ..\..` while within the make folder
  2. Run `.\setup.ps1 -m` while within the FlightSupervisor folder

## Contact

James Horner - JamesHorner@cmail.carleton.ca or jwehorner@gmail.com

Tanner Trautrim - trautrim.tbt@outlook.com

Project Link: [https://github.com/SimulationEverywhere-Models/FlightSupervisor](https://github.com/SimulationEverywhere-Models/FlightSupervisor)

## Acknowledgements
