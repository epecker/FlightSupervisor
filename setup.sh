#!/bin/bash

#########################
###  Parse Arguments  ###
#########################
while getopts "hbiumx" option; do
    case ${option} in
        h)
            help="true"
            ;;
        b)
            build_submodules="true"
            ;;
        i)
            initialize_submodules="true"
            ;;
        u)
            update_submodules="true"
            ;;
        m)
            generate_make="true"
            ;;
        x)
            generate_xcode="true"
            ;;
        \?)
            echo "Invalid Option -$option. Use -h for help." 2> /dev/null
            exit 3
            ;;
        :)
            echo "Missing arguments for -$option. Use -h for help."
            exit 3
            ;;
    esac
done


###################
###  Functions  ###
###################

printTitle() {
    printf "  ____                              _                \n"
    printf " / ___| _   _ _ __   ___ _ ____   _(_)___  ___  _ __ \n"
    printf " \___ \| | | | '_ \ / _ \ '__\ \ / / / __|/ _ \| '__|\n"
    printf "  ___) | |_| | |_) |  __/ |   \ V /| \__ \ (_) | |   \n"
    printf " |____/ \__,_| .__/ \___|_|    \_/ |_|___/\___/|_|   \n"
    printf "             |_|                                     \n\n"
}

usage() {
    if [ "$1" != "init" ] && [ "$1" != "rebuild" ] || [ -z "$1" ]; then
        printf " -h \t Access this help menu\n"
        printf " -b \t Build the submodels\n"
        printf " -i \t Initialize the submodules\n"
        printf " -u \t Update the submodules\n"
        printf " -m \t Generate the Supervisors make files\n"
        printf " -x \t Generate the Supervisors Xcode files\n"
        printf "\n"
        printf " Examples:\n"
        printf "   setup -h\n"
        printf "   setup -i\n"
        printf "   setup -b\n"
        printf "   setup -ib\n"
        printf "   setup -ibm\n"
        printf "   setup -ibx\n"
        printf "   setup -mx\n"
        exit 0
    fi
}

initSubmodules() {
    printf " ###   Deinitialize Submodules   ###\n"

    git submodule deinit --all > /dev/null 2>&1

    if [ $? -ne 0 ]; then
        printf "   Deinitialize Failed\n"
        printf "   If you would like to discard your changes\n"
        printf "   to the submodules manually run:\n"
        printf "   git submodule deinit --all -f\n"
        printf "   Then rerun the script"
        exit 1
    fi

    printf "   Complete\n\n"
    printf " ###   Initialize Submodules   ###\n"

    git submodule init > /dev/null 2>&1
    git submodule update > /dev/null 2>&1

    if [ $? -ne 0 ]; then
        printf "   Initialization failed\n"
        printf "   Check your folder permission for the submodules\n"
        printf "   Verify that you have rights to access the submodule"
        printf "   repository(ies)"
        exit 1
    fi

    printf "   Complete\n\n"
}

updateSubmodules() {
    printf " ###   Update Submodules   ###\n"

    git submodule update --init > /dev/null 2>&1

    if [ $? -ne 0 ]; then
        printf "   Update Failed\n"
        printf "   If you would like to discard your changes\n"
        printf "   to the submodules manually run:\n"
        printf "   git submodule deinit --all -f\n"
        printf "   Then rerun the script\n"
        printf "   Verify that you have rights to access the submodule"
        printf "   repository(ies)"
        exit 1
    fi

    printf "   Complete\n\n"
}

rudp() {
    printf " ###   Create RUDP Build Directory   ###\n"

    if [ ! -d "./deps/RUDP/build" ]; then
        mkdir ./deps/RUDP/build
        if [ $? -ne 0 ]; then
            printf "   Failed to create RUDP build directory\n"
            exit 1
        fi
    fi

    printf "   Complete\n\n"
    printf " ###   Generate RUDP Build Files   ###\n"

    cd ./deps/RUDP/build
    cmake .. | sed "s/^/   /"

    if [ $? -ne 0 ]; then
        exit 1
    fi

    printf "\n ###   Make RUDP   ###\n"

    make rudp | sed "s/^/   /"

    if [ $? -ne 0 ]; then
        exit 1
    fi

    cd ../../..
}

generate_make_build() {
    printf " ###   Create Supervisor Make Build Directory   ###\n"

    if [ ! -d "./build/make" ]; then
        mkdir -p ./build/make
        if [ $? -ne 0 ]; then
            printf "   Failed to create supervisor make directory\n"
            exit 1
        fi
    fi

    printf "   Complete\n\n"

    printf " ###   Generate Supervisor Make Files   ###\n"

    cd ./build/make
    cmake ../.. | sed "s/^/   /"

    if [ $? -ne 0 ]; then
        exit 1
    fi

    printf "   Complete\n\n"
    cd ../..
}

generate_xcode_build() {
    printf " ###   Create Supervisor Xcode Build Directory   ###\n"

    if [ ! -d "./build/xcode" ]; then
        mkdir -p ./build/xcode
        if [ $? -ne 0 ]; then
            printf "   Failed to create supervisor Xcode directory\n"
            exit 1
        fi
    fi

    printf "   Complete\n\n"
    printf " ###   Generate Supervisor Xcode Files   ###\n"

    cd ./build/xcode
    cmake -G Xcode ../.. | sed "s/^/   /"

    if [ $? -ne 0 ]; then
        exit 1
    fi

    printf "   Complete\n\n"
    cd ../..
}


##############
###  Main  ###
##############

printTitle
[[ $# -eq 0 ]] && usage && exit 0;
[[ "$help" == "true" ]] && usage && exit 0
[[ "$initialize_submodules" == "true" ]] && initSubmodules
[[ "$update_submodules" == "true" ]] && updateSubmodules
[[ "$build_submodules" == "true" ]] && rudp
[[ "$generate_make" == "true" ]] && generate_make_build
[[ "$generate_xcode" == "true" ]] && generate_xcode_build
