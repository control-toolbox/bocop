set problem_dir=%~dp0
set bocop_root_path=..\..\..

rmdir /s /q build
mkdir build
cd build

cmake -G "NMake Makefiles" -DPROBLEM_DIR=%problem_dir%  %bocop_root_path%
nmake

cd.. 
