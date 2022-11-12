set workdir=%cd%

cd ARMS
git pull
cd src
xcopy "ARMS" %workdir%\src\ARMS
cd ..
cd include
xcopy "ARMS" %workdir%\include\ARMS
cd %workdir%