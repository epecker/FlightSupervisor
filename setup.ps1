<#########################
 ###  Parse Arguments  ###
 #########################>
 param (
     [switch]$h,
     [switch]$b,
     [switch]$i,
     [switch]$u,
     [switch]$m
     # [switch]$x
 )

 if ($PSBoundParameters.ContainsKey('h')) {
     $help = $true
 }

 if ($PSBoundParameters.ContainsKey('b')) {
     $build_submodules = $true
 }
 
 if ($PSBoundParameters.ContainsKey('i')) {
     $initialize_submodules = $true
}

if ($PSBoundParameters.ContainsKey('u')) {
    $update_submodules = $true
}

if ($PSBoundParameters.ContainsKey('m')) {
    $generate_make = $true
}

<#if ($PSBoundParameters.ContainsKey('x')) {
    $generate_xcode = $true
}#>

if (!$help -and !$build_submodules -and !$initialize_submodules -and !$update_submodules -and !$generate_make) {
     Write-Host "Missing or unknown arguments. Use -h for help."
     exit 3
}

<###################
 ###  Functions  ###
 ###################>
function printTitle {
    
    param()

    Write-Host "  ____                              _                "
    Write-Host " / ___| _   _ _ __   ___ _ ____   _(_)___  ___  _ __ "
    Write-Host " \___ \| | | | '_ \ / _ \ '__\ \ / / / __|/ _ \| '__|"
    Write-Host "  ___) | |_| | |_) |  __/ |   \ V /| \__ \ (_) | |   "
    Write-Host " |____/ \__,_| .__/ \___|_|    \_/ |_|___/\___/|_|   "
    Write-Host "             |_|                                     `n"
}

function usage {

    param()

    Write-Host " -h `t Access this help menu"
    Write-Host " -b `t Build the submodels"
    Write-Host " -i `t Initialize the submodules"
    Write-Host " -u `t Update the submodules"
    Write-Host " -m `t Generate the Supervisors make files"
    # Write-Host " -x `t Generate the Supervisors Xcode files"
    Write-Host "`n"
    Write-Host " Examples:"
    Write-Host "   setup -h"
    Write-Host "   setup -i"
    Write-Host "   setup -b"
    Write-Host "   setup -i -b"
    Write-Host "   setup -i -b -m"
    # Write-Host "   setup -i -b -x"
    # Write-Host "   setup -m -x"
    Write-Host "`n"
}

<##############################
 ###  Init git submodules   ###
 ##############################>
function initSubmodules {

    param()

    Write-Host " ###   Deinitialize Submodules   ###"

    git submodule deinit --all > $null

    if ( $? -eq $false ) {
        Write-Host "   Deinitialize Failed"
        Write-Host "   If you would like to discard your changes"
        Write-Host "   to the submodules manually run:"
        Write-Host "   git submodule deinit --all -f"
        Write-Host "   Then rerun the script"
        exit 1
    }

    Write-Host "   Complete"
    Write-Host " ###   Initialize Submodules   ###"

    git submodule init   > $null
    git submodule update --remote --recursive > $null

    if ( $? -eq $false ) {
        Write-Host "   Initialization failed"
        Write-Host "   Check your folder permission for the submodules"
        Write-Host "   Verify that you have rights to access the submodule"
        Write-Host "   repository(ies)"
        exit 1
    }

    Write-Host "   Complete`n"
}

<###############################
 ###  Update git submodules  ###
 ###############################>
function updateSubmodules {

    param()

    Write-Host " ###   Update Submodules   ###"

    git submodule update --init > $null

    if ( $? -eq $false ) {
        Write-Host "   Update Failed"
        Write-Host "   If you would like to discard your changes"
        Write-Host "   to the submodules manually run:"
        Write-Host "   git submodule deinit --all -f"
        Write-Host "   Then rerun the script"
        Write-Host "   Verify that you have rights to access the submodule"
        Write-Host "   repository(ies)"
        exit 1
    }

    Write-Host "   Complete`n"
}

<##############################
 ###  Build RUDP submodule  ###
 ##############################>
function rudp {

    param()

    Write-Host "###   Create RUDP Build Directory   ###"

    if (Test-Path -Path "./deps/RUDP/build") {
        Write-Host "RUDP build directory already exists"
    } else {
        New-Item -Path "./deps/RUDP/build" -Type Directory
        if ($? -eq $false) {
            Write-Host "Failed to create RUDP build directory"
            exit 1
        }
    }

    Write-Host "Complete`n"
    Write-Host "###   Generate RUDP Build Files   ###"

    cd ./deps/RUDP/build
    cmake ..

    if ($? -eq $false ) {
        exit 1
    }

    Write-Host "`n ###   Make RUDP   ###`n"

    cmake --build .

    if ($? -eq $false ){
        exit 1
    }

    cd ../../..
}

<##########################
 ###  Build Supervisor  ###
 ##########################>
function generate_make_build {

    param()

    Write-Host " ###   Create Supervisor Make Build Directory   ###"

    if (Test-Path -Path "./build/make") {
        Write-Host "Supervisor build directory already exists"
    } else {
        New-Item -Path "./build/make" -Type Directory
        if ($? -eq $false) {
            Write-Host "   Failed to create supervisor make directory"
            exit 1
        }
    }

    Write-Host "   Complete`n"

    Write-Host " ###   Generate Supervisor Make Files   ###"

    cd ./build/make
    cmake ../..

    if ($? -eq $false ) {
        exit 1
    }

    Write-Host "`n ###   Make Supervisor   ###`n"
    cmake --build .

    if ($? -eq $false ){
        exit 1
    }

    Write-Host "   Complete`n"
    cd ../..
}

<################################
 ###  Build Supervisor Xcode  ###
 ################################>
<#function generate_xcode_build {

    param()

    Write-Host " ###   Create Supervisor Xcode Build Directory   ###"

    if (Test-Path -Path "./build/xcode") {
        Write-Host "Supervisor build directory already exists"
    } else {
        # mkdir ./build/xcode
        New-Item -Path "./build/xcode" -Type Directory
        if ($? -eq $false) {
            Write-Host "   Failed to create supervisor Xcode directory"
            exit 1
        }
    }

    Write-Host "   Complete`n"
    Write-Host " ###   Generate Supervisor Xcode Files   ###"

    cd ./build/xcode
    cmake -G Xcode ../..

    if ($? -eq $false ) {
        exit 1
    }

    Write-Host "   Complete`n"
    cd ../..
}#>

<##############
 ###  Main  ###
 ##############>

printTitle
if( $help -eq $true ) {
    usage
    exit 0
}
if ($initialize_submodules -eq $true) {
    initSubmodules
}
if ($update_submodules -eq $true) {
    updateSubmodules
}
if ($build_submodules -eq $true) {
    rudp
}
if ($generate_make -eq $true) {
    generate_make_build
}
<#if ($generate_xcode -eq $true) {
    generate_xcode_build
}#>