@ECHO OFF
REM Performs the steps needed to create the final lake mesh from the already raytraced data
REM Useful for generating multiple sets of meshes and sonar points with different clustering resolutions and noise filtering levels.
REM Make sure that the paths to binaries inside all of the batch files point to the correct tools in your machine!!
SET dtmerge=X:/ENDURANCE/dttools/trunk/bin/win-x86-vs10-release/dtmerge.exe 
SET meshlab="C:\Program Files\VCG\MeshLab\meshlabserver.exe"
SET dtpoisson=X:/ENDURANCE/dttools/trunk/bin/win-x86-vs10-debug/dtpoisson.exe 

SET postfix=1m4x

echo ---- Building AllData Mesh --------------------
%dtmerge% bonney-all.dtmerge.cfg
cd bonney
copy bonney-points.csv %postfix%\bonney-all-points-%postfix%.asc
cd ..
REM %meshlab% -i %postfix%\bonney-all-points-%postfix%.asc -o bonney/%postfix%/bonney-all-points-%postfix%.ply -s bonney-points.mlx

%dtmerge% bonney.dtmerge.cfg

cd bonney
copy bonney.xyz %postfix%\bonney-all-points-%postfix%.csv
cd ..

%dtpoisson% bonney-lowres.dtpoisson.cfg
%meshlab% -i bonney/bonney.ply -o bonney/%postfix%/bonney-all-lowres-%postfix%.ply -s bonney-mesh.mlx
%dtpoisson% bonney-hires.dtpoisson.cfg
%meshlab% -i bonney/bonney.ply -o bonney/%postfix%/bonney-all-hires-%postfix%.ply -s bonney-mesh.mlx
