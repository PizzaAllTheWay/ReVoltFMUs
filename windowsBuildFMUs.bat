If exist "build" ( 
    echo "build folder exists"
) Else ( 
    mkdir build
)
cd build
cmake .. --fresh
cmake --build . --config Release
