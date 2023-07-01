#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
cmake -B build
cmake --build build

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
# build/PA1 testcases/scene01_basic.txt output/scene01.bmp
# build/PA1 testcases/scene02_cube.txt output/scene02.bmp
# build/PA1 testcases/scene03_sphere.txt output/scene03.bmp
# build/PA1 testcases/scene04_axes.txt output/scene04.bmp
# build/PA1 testcases/scene05_bunny_200.txt output/scene05.bmp
# build/PA1 testcases/scene06_bunny_1k.txt output/scene06.bmp
# build/PA1 testcases/scene07_shine.txt output/scene07.bmp

# build/PA1 pt testcases/testscene2.txt output/testscene2.bmp 50 > te2.txt
# build/PA1 pt testcases/testscene6.txt output/testscene6.bmp 10 > te6.txt
# build/PA1 pt testcases/testscene7.txt output/testscene7.bmp 10 > te7.txt
# build/PA1 pt testcases/basic.txt output/basic.bmp 5000 > ba.txt
# build/PA1 pt testcases/water.txt output/water.bmp 10 > wa.txt
# build/PA1 pt testcases/mov.txt output/mov.bmp 50 > mo.txt
# build/PA1 pt testcases/testscene5.txt output/testscene5.bmp nodebug > te5.txt
# build/PA1 pt testcases/testscene4.txt output/testscene4.bmp nodebug > te4.txt
# build/PA1 pt testcases/wineglasses.txt output/wineglasses.bmp nodebug > win.txt
# build/PA1 pt testcases/vase.txt output/vase.bmp 500 > va.txt
# build/PA1 pt testcases/lagenaria.txt output/lagenaria.bmp 500 > lag.txt
# build/PA1 pt testcases/dao.txt output/dao.bmp 1000 > da.txt
# build/PA1 pt testcases/dof.txt output/dof.bmp 5000 > do.txt
# build/PA1 pt testcases/basic.txt output/basic.bmp 10000 > ba.txt
# build/PA1 pt testcases/mov.txt output/mov.bmp 5000 > mo.txt
# build/PA1 pt testcases/chess_bunny.txt output/chess_bunny.bmp 1000 > cb.txt
# build/PA1 pt testcases/syc.txt output/syc.bmp 20 > sy.txt
build/PA1 pt testcases/final.txt output/final.bmp 10 > fin.txt
# build/PA1 pt testcases/scene09_norm.txt output/scene09_norm.bmp 10 > s9.txt