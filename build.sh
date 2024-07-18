#!/bin/bash
cd src/utils/include/utils/
# Run make and capture any error
if ! make; then
    # Print error message in red
    echo -e "\e[31mCompilation failed due to link of gfortran. You should refer to https://askubuntu.com/questions/276892/cannot-find-lgfortran or use ubuntu20 instead\e[0m"
    echo -e "\e[31mCompilation failed due to link of gfortran. You should refer to https://askubuntu.com/questions/276892/cannot-find-lgfortran or use ubuntu20 instead\e[0m"
    echo -e "\e[31mCompilation failed due to link of gfortran. You should refer to https://askubuntu.com/questions/276892/cannot-find-lgfortran or use ubuntu20 instead\e[0m"
    echo -e "\e[31mCompilation failed due to link of gfortran. You should refer to https://askubuntu.com/questions/276892/cannot-find-lgfortran or use ubuntu20 instead\e[0m"
    echo -e "\e[31mCompilation failed due to link of gfortran. You should refer to https://askubuntu.com/questions/276892/cannot-find-lgfortran or use ubuntu20 instead\e[0m"
    echo -e "\e[31mCompilation failed due to link of gfortran. You should refer to https://askubuntu.com/questions/276892/cannot-find-lgfortran or use ubuntu20 instead\e[0m"
    exit 1
else
    echo "Compilation succeeded."
fi

make clean &&sleep 1
cd ../../../../
catkin_make -j20 -DPYTHON_EXECUTABLE=/usr/bin/python3