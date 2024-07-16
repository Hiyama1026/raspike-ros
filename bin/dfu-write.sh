#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Error: Exactly one argument is required." >&2
    echo ""
    echo "- For WSL2 ->'win'"
    echo "- For Linux(not WSL) -> 'lin'"
    echo "- Clean state -> 'clean'"
    exit 1
fi

if [ ! "$1" == "lin" ] && [ ! "$1" == "win" ] && [ ! "$1" == "clean" ]; then
    echo "Error: Argument must be 'lin' or 'win'."
    echo ""
    echo "- For WSL2 ->'win'"
    echo "- For Linux(not WSL) -> 'lin'"
    echo "- Clean state -> 'clean'"
    exit 1
fi

if [ ! -d "tools" ]; then
    mkdir tools
fi

if [ "$1" == "clean" ]; then
    rm -rf ./tools
fi

if [ "$1" == "lin" ]; then
    # Check if pydfu.py is installed
    if [ -d "tools/micropython" ]; then
        if [ ! -e "tools/micropython/tools/pydfu.py" ]; then
            echo "Error: Cannot find ./tools/micropython/tools/pydfu.py" >&2
            echo "       Prease remove ./tools/micropython ($ rm -rf ./tools/micropython)." >&2
            exit 1
        fi
    else
        git clone https://github.com/micropython/micropython.git -b v1.23.0 ./tools/micropython
    fi

    # dfu write
    sudo python3 tools/micropython/tools/pydfu.py -u asp.dfu --vid 0x0694 --pid 0x0008
fi


if [ "$1" == "win" ]; then

    # Check if dfu-util.exe is installed
    if [ -f "tools/dfu-util-0.11-binaries.tar.xz" ]; then
        if [ -d "tools/dfu-util-0.11-binaries" ]; then
            if [ ! -e "tools/dfu-util-0.11-binaries/win64/dfu-util.exe" ]; then
                echo "Error: Cannot find ./tools/dfu-util-0.11-binaries/win64/dfu-util.exe" >&2
                echo "       Prease remove ./tools/dfu-util-0.11-binaries ($ rm -rf ./tools/dfu-util-0.11-binaries)." >&2
                exit 1
            fi
        else
            # Extract dfu-util-0.11-binaries.tar.xz
            tar Jxfv ./tools/dfu-util-0.11-binaries.tar.xz -C ./tools
        fi
    elif [ -d "tools/dfu-util-0.11-binaries" ]; then
        if [ ! -e "tools/dfu-util-0.11-binaries/win64/dfu-util.exe" ]; then
            echo "Error: Cannot find ./tools/dfu-util-0.11-binaries/win64/dfu-util.exe" >&2
            echo "       Prease remove ./tools/dfu-util-0.11-binaries ($ rm -rf ./tools/dfu-util-0.11-binaries)." >&2
            exit 1
        fi
    else
        # Download and extract dfu-util-0.11-binaries.tar.xz
        curl -L -o tools/dfu-util-0.11-binaries.tar.xz https://dfu-util.sourceforge.net/releases/dfu-util-0.11-binaries.tar.xz
        tar Jxfv ./tools/dfu-util-0.11-binaries.tar.xz -C ./tools
    fi

    # dfu write
    ./tools/dfu-util-0.11-binaries/win64/dfu-util.exe -a 0 -d 0x0694:0x0008 -s 0x8008000 -R -D asp.bin
fi