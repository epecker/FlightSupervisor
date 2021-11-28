## About The Project

## Built With

* Boost<span>.</span>org
* CMake
* SimulationEverywhere/Cadmium
* SimulationEverywhere/NDTime

## Prerequisites

Note: CMake version 3.22 support up to Boost version 1.75. Do not install a version of boost newer than 1.75.

### GNU/Linux - Ubuntu

Using a terminal perform the following.

* Update the Ubuntu repositories

    ```
    apt update
    ```

* Upgrade the Ubuntu System

    ```
    apt upgrade
    ```

* Install the ubuntu development tools

    ```
    apt install build-essential
    ```

* Install Boost version 1.74

    ```
    apt install libboost1.74-all-dev
    ```

* Install CMake via snap

    ```
    snap install cmake –classic
    ```

* Install Git

    ```
    apt install git
    ```

### MacOS - XCode with Homebrew

Using a terminal perform the following.

* Install cmake
    ```
    brew install cmake
    ```

* Find a version of boost up to and including version 1.75  
    ```
    brew search boost
    ```

* Install a suitable version of boost. Replace 1.75 with a version listed above.
    ```
    brew search boost@1.75
    ```

* XCode bundles the git binary.
    * Install Git anyways
        ```
        brew install git
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

## Installation

## Usage

## Roadmap

## License

## Contact

James Horner - JamesHorner@cmail.carleton.ca or jwehorner@gmail.com

Tanner Trautrim - trautrim.tbt@outlook.com

Project Link: [https://github.com/SimulationEverywhere-Models/FlightSupervisor](https://github.com/SimulationEverywhere-Models/FlightSupervisor)

## Acknowledgements
