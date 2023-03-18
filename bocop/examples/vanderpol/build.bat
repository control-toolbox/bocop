set problem_dir=%~dp0
set bocop_root_path=..\..\..

dir %bocop_root_path%
dir

rmdir /s /q build
mkdir build
cd build
echo "Go to build subdir"
dir

echo "cmake step"
cmake -G "NMake Makefiles" -DPROBLEM_DIR=%problem_dir% %bocop_root_path%
echo "nmake step"
nmake

cd.. 
