v 20110115 2
C 22500 40000 0 0 0 title-A3.sym
C 36100 46200 1 0 1 in-1.sym
{
T 36100 46500 5 10 0 0 0 6 1
device=INPUT
T 36900 46200 5 10 1 1 0 6 1
refdes=VCC_3V3
}
C 36100 44000 1 0 1 in-1.sym
{
T 36100 44300 5 10 0 0 0 6 1
device=INPUT
T 36500 44000 5 10 1 1 0 6 1
refdes=GND
}
C 30400 43900 1 0 0 ADuM3160-1.sym
{
T 32900 46650 5 10 1 1 0 6 1
device=ADuM3160
T 30700 46650 5 10 1 1 0 0 1
refdes=U1
T 30700 47950 5 10 0 0 0 0 1
footprint=SO16W
}
N 25400 46300 30400 46300 4
N 25400 44100 30400 44100 4
C 27800 45000 1 180 0 resistor-2.sym
{
T 27400 44650 5 10 0 0 180 0 1
device=RESISTOR
T 27000 45200 5 10 1 1 180 6 1
refdes=R1
T 27300 45200 5 10 1 1 180 6 1
value=24/1%
T 27800 45000 5 10 0 1 0 0 1
footprint=0805
}
N 30400 44900 27800 44900 4
N 30400 44700 27100 44700 4
N 25400 44700 26200 44700 4
N 25400 44900 26900 44900 4
N 30400 45300 30100 45300 4
N 30100 45300 30100 46100 4
N 30200 45700 30400 45700 4
N 28600 46100 30400 46100 4
N 30200 45700 30200 46100 4
C 28200 45100 1 90 0 capacitor-1.sym
{
T 27500 45300 5 10 0 0 90 0 1
device=CAPACITOR
T 27900 45900 5 10 1 1 0 6 1
refdes=C1
T 27300 45300 5 10 0 0 90 0 1
symversion=0.1
T 27900 45700 5 10 1 1 0 6 1
value=>100n/low ESR
T 28200 45100 5 10 0 0 0 0 1
footprint=0805
}
C 27100 44800 1 180 0 resistor-2.sym
{
T 26700 44450 5 10 0 0 180 0 1
device=RESISTOR
T 26300 44500 5 10 1 1 180 6 1
refdes=R2
T 26600 44500 5 10 1 1 180 6 1
value=24/1%
T 27100 44800 5 10 0 1 0 0 1
footprint=0805
}
N 30400 44300 30100 44300 4
N 30100 44300 30100 44100 4
C 28800 45100 1 90 0 capacitor-1.sym
{
T 28100 45300 5 10 0 0 90 0 1
device=CAPACITOR
T 28700 45900 5 10 1 1 0 0 1
refdes=C2
T 27900 45300 5 10 0 0 90 0 1
symversion=0.1
T 28700 45700 5 10 1 1 0 0 1
value=>100n/low ESR
T 28800 45100 5 10 0 0 0 0 1
footprint=0805
}
N 28600 46000 28600 46100 4
N 28600 45100 28600 44100 4
N 28000 45100 28000 44100 4
N 28000 46000 28000 46300 4
C 37200 45000 1 180 0 resistor-2.sym
{
T 36800 44650 5 10 0 0 180 0 1
device=RESISTOR
T 36400 45200 5 10 1 1 180 6 1
refdes=R3
T 36700 45200 5 10 1 1 180 6 1
value=24/1%
T 37200 45000 5 10 0 1 0 0 1
footprint=0805
}
N 37700 44700 36500 44700 4
C 36500 44800 1 180 0 resistor-2.sym
{
T 36100 44450 5 10 0 0 180 0 1
device=RESISTOR
T 35700 44500 5 10 1 1 180 6 1
refdes=R4
T 36000 44500 5 10 1 1 180 6 1
value=24/1%
T 36500 44800 5 10 0 1 0 0 1
footprint=0805
}
N 35600 44700 33200 44700 4
N 33200 44900 36300 44900 4
N 33200 46100 34900 46100 4
N 33200 45700 33400 45700 4
N 33400 45700 33400 46100 4
N 33200 45300 33500 45300 4
N 33500 45300 33500 46100 4
N 33200 44300 33500 44300 4
N 33500 44100 33500 44300 4
N 33200 44100 35500 44100 4
N 33200 46300 35500 46300 4
N 33600 46300 33600 46100 4
C 35100 45100 1 90 0 capacitor-1.sym
{
T 34400 45300 5 10 0 0 90 0 1
device=CAPACITOR
T 34800 45900 5 10 1 1 0 6 1
refdes=C3
T 34200 45300 5 10 0 0 90 0 1
symversion=0.1
T 34800 45700 5 10 1 1 0 6 1
value=>100n/low ESR
T 35100 45100 5 10 0 0 0 0 1
footprint=0805
}
C 35700 45100 1 90 0 capacitor-1.sym
{
T 35000 45300 5 10 0 0 90 0 1
device=CAPACITOR
T 35600 45900 5 10 1 1 0 0 1
refdes=C4
T 34800 45300 5 10 0 0 90 0 1
symversion=0.1
T 35600 45700 5 10 1 1 0 0 1
value=>100n/low ESR
T 35700 45100 5 10 0 0 0 0 1
footprint=0805
}
N 34900 45100 34900 44100 4
N 35500 46300 35500 46000 4
N 35500 45100 35500 44100 4
N 34900 46100 34900 46000 4
C 38300 44800 1 0 1 in-1.sym
{
T 38300 45100 5 10 0 0 0 6 1
device=INPUT
T 38300 44800 5 10 1 1 0 0 1
refdes=D-
}
C 38300 44600 1 0 1 in-1.sym
{
T 38300 44900 5 10 0 0 0 6 1
device=INPUT
T 38300 44600 5 10 1 1 0 0 1
refdes=D+
}
N 37200 44900 37700 44900 4
C 24800 46200 1 0 0 in-1.sym
{
T 24800 46500 5 10 0 0 0 0 1
device=INPUT
T 24800 46200 5 10 1 1 0 6 1
refdes=USB_5V
}
C 24800 44000 1 0 0 in-1.sym
{
T 24800 44300 5 10 0 0 0 0 1
device=INPUT
T 24800 44000 5 10 1 1 0 6 1
refdes=USB_GND
}
C 25400 44800 1 0 1 out-1.sym
{
T 25400 45100 5 10 0 0 0 6 1
device=OUTPUT
T 24000 44800 5 10 1 1 0 0 1
refdes=USB_D-
}
C 25400 44600 1 0 1 out-1.sym
{
T 25400 44900 5 10 0 0 0 6 1
device=OUTPUT
T 24000 44600 5 10 1 1 0 0 1
refdes=USB_D+
}
T 22800 46800 2 20 1 0 0 0 1
TODO: TVS Diodes/ESD Supressors