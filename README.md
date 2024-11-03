# Surface Fitting

Run in cmd

`SurfaceFitting.exe [obj] [xslice] [yslice] [zslice]`

`[obj]`：Obj model

`[xslice]`：The number of slices on x-axis (default 1).

`[yslice]`：The number of slices on y-axis (default 1).

`[zslice]`：The number of slices on z-axis (default 1).

You will get `param.txt` after running the program.

Each line of `param.txt` consists of the left bottom corner point and the right top corner point of a voxel, the parameters of the ellipsoid function.

You can use `display.py` to show ellipsoids or view them in the program.

