#!/bin/sh

DTRT="mpirun -machinefile lyra -np 128 ../bin/linux-gcc-debug/mpidtrt "

#eval "$DTRT bonney-08-dive05.dtrt.cfg"
eval "$DTRT bonney-08-dive07.dtrt.cfg"
eval "$DTRT bonney-08-dive08.dtrt.cfg"
eval "$DTRT bonney-08-dive09.dtrt.cfg"
eval "$DTRT bonney-08-dive10.dtrt.cfg"
eval "$DTRT bonney-08-dive12.dtrt.cfg"
eval "$DTRT bonney-08-dive13.dtrt.cfg"
eval "$DTRT bonney-08-dive15.dtrt.cfg"
eval "$DTRT bonney-08-dive16.dtrt.cfg"
eval "$DTRT bonney-08-dive17.dtrt.cfg"
eval "$DTRT bonney-08-dive18.dtrt.cfg"

eval "$DTRT bonney-09-dive13.dtrt.cfg"
eval "$DTRT bonney-09-dive17.dtrt.cfg"
eval "$DTRT bonney-09-dive18.dtrt.cfg"
eval "$DTRT bonney-09-dive19.dtrt.cfg"
eval "$DTRT bonney-09-dive20.dtrt.cfg"
eval "$DTRT bonney-09-dive21.dtrt.cfg"
eval "$DTRT bonney-09-dive22.dtrt.cfg"
eval "$DTRT bonney-09-dive23.dtrt.cfg"
eval "$DTRT bonney-09-dive24.dtrt.cfg"
eval "$DTRT bonney-09-dive25.dtrt.cfg"
eval "$DTRT bonney-09-dive26.dtrt.cfg"
eval "$DTRT bonney-09-dive27.dtrt.cfg"

