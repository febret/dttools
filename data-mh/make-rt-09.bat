SET dtrt=X:/ENDURANCE/dttools/trunk/bin/win-x86-vs10-release/dtrt.exe 


FOR %%D IN (13 17 18 19 20 21 22 23 24 25 26 27) DO %dtrt% bonney-09-dive%%D.dtrt.cfg >>&2 bonney-dtrt.log

