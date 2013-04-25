SET dtrt=D:/Workspace/ENDURANCE/dttools/trunk/bin/win-x86-vs10-release/dtrt.exe 
FOR %%D IN (05 07 08 09 10 12 13) DO %dtrt% bonney-08-dive%%D.dtrt.cfg
