v 20110115 2
C 42000 41000 0 0 0 title-A3.sym
C 48500 49800 1 0 0 gnd-3.sym
{
T 48800 49950 5 10 1 1 0 5 1
net=GND:1
}
C 48600 51100 1 0 0 generic-power.sym
{
T 48800 51350 5 10 1 1 0 3 1
net=VCC_2V5:1
}
C 48600 44400 1 90 0 resistor-variable-2.sym
{
T 48700 44900 5 10 1 1 0 0 1
refdes=R6
T 47700 45200 5 10 0 1 90 0 1
device=VARIABLE_RESISTOR
}
C 48200 44000 1 90 0 capacitor-1.sym
{
T 47500 44200 5 10 0 0 90 0 1
device=CAPACITOR
T 47900 44200 5 10 1 1 0 6 1
refdes=C3
T 47300 44200 5 10 0 0 90 0 1
symversion=0.1
T 47900 44000 5 10 1 1 0 6 1
value=100n
T 48200 44000 5 10 0 0 0 0 1
footprint=0805
}
C 48200 43600 1 0 0 gnd-3.sym
{
T 48500 43750 5 10 1 1 0 5 1
net=GND:1
}
N 48000 44000 48500 44000 4
N 48500 44400 48500 44000 4
C 48300 45300 1 0 0 generic-power.sym
{
T 48500 45550 5 10 1 1 0 3 1
net=VCC_2V5:1
}
N 47100 44900 48000 44900 4
C 52100 44400 1 90 0 resistor-variable-2.sym
{
T 52200 44900 5 10 1 1 0 0 1
refdes=R10
T 51200 45200 5 10 0 1 90 0 1
device=VARIABLE_RESISTOR
}
C 51700 44000 1 90 0 capacitor-1.sym
{
T 51000 44200 5 10 0 0 90 0 1
device=CAPACITOR
T 51400 44200 5 10 1 1 0 6 1
refdes=C6
T 50800 44200 5 10 0 0 90 0 1
symversion=0.1
T 51400 44000 5 10 1 1 0 6 1
value=100n
T 51700 44000 5 10 0 0 0 0 1
footprint=0805
}
C 51700 43600 1 0 0 gnd-3.sym
{
T 52000 43750 5 10 1 1 0 5 1
net=GND:1
}
N 51500 44000 52000 44000 4
N 52000 44400 52000 44000 4
C 51800 45300 1 0 0 generic-power.sym
{
T 52000 45550 5 10 1 1 0 3 1
net=VCC_2V5:1
}
N 50100 44900 51500 44900 4
C 45300 44400 1 90 0 resistor-variable-2.sym
{
T 45400 44900 5 10 1 1 0 0 1
refdes=R4
T 44400 45200 5 10 0 1 90 0 1
device=VARIABLE_RESISTOR
}
C 44900 44000 1 90 0 capacitor-1.sym
{
T 44200 44200 5 10 0 0 90 0 1
device=CAPACITOR
T 44600 44200 5 10 1 1 0 6 1
refdes=C2
T 44000 44200 5 10 0 0 90 0 1
symversion=0.1
T 44600 44000 5 10 1 1 0 6 1
value=100n
T 44900 44000 5 10 0 0 0 0 1
footprint=0805
}
C 44900 43600 1 0 0 gnd-3.sym
{
T 45200 43750 5 10 1 1 0 5 1
net=GND:1
}
N 44700 44000 45200 44000 4
N 45200 44400 45200 44000 4
C 45000 45300 1 0 0 generic-power.sym
{
T 45200 45550 5 10 1 1 0 3 1
net=VCC_2V5:1
}
C 55800 45400 1 0 0 switch-pb-1.sym
{
T 57000 45600 5 10 1 1 0 6 1
refdes=SW5
}
C 55800 45500 1 180 0 resistor-2.sym
{
T 55400 45150 5 10 0 0 180 0 1
device=RESISTOR
T 55000 45700 5 10 1 1 180 6 1
refdes=R13
T 55500 45700 5 10 1 1 180 6 1
value=1k
}
C 55800 45100 1 180 0 capacitor-1.sym
{
T 55600 44400 5 10 0 0 180 0 1
device=CAPACITOR
T 54900 44700 5 10 1 1 0 0 1
refdes=C9
T 55600 44200 5 10 0 0 180 0 1
symversion=0.1
T 55500 44700 5 10 1 1 0 0 1
value=100n
T 55800 45100 5 10 0 0 90 0 1
footprint=0805
}
N 54900 44900 54900 45400 4
N 57000 44900 57000 45400 4
N 57000 44900 55800 44900 4
C 57200 44300 1 0 0 gnd-3.sym
{
T 57500 44450 5 10 1 1 0 5 1
net=GND:1
}
C 55800 46900 1 0 0 switch-pb-1.sym
{
T 57000 47100 5 10 1 1 0 6 1
refdes=SW4
}
C 55800 47000 1 180 0 resistor-2.sym
{
T 55400 46650 5 10 0 0 180 0 1
device=RESISTOR
T 55000 47200 5 10 1 1 180 6 1
refdes=R12
T 55500 47200 5 10 1 1 180 6 1
value=1k
}
C 55800 46600 1 180 0 capacitor-1.sym
{
T 55600 45900 5 10 0 0 180 0 1
device=CAPACITOR
T 54900 46200 5 10 1 1 0 0 1
refdes=C8
T 55600 45700 5 10 0 0 180 0 1
symversion=0.1
T 55500 46200 5 10 1 1 0 0 1
value=100n
T 55800 46600 5 10 0 0 90 0 1
footprint=0805
}
N 54900 46900 54900 46400 4
N 57000 46400 57000 46900 4
N 57000 46400 55800 46400 4
C 55800 48400 1 0 0 switch-pb-1.sym
{
T 57000 48600 5 10 1 1 0 6 1
refdes=SW3
}
C 55800 48500 1 180 0 resistor-2.sym
{
T 55400 48150 5 10 0 0 180 0 1
device=RESISTOR
T 55000 48700 5 10 1 1 180 6 1
refdes=R11
T 55500 48700 5 10 1 1 180 6 1
value=1k
}
C 55800 48100 1 180 0 capacitor-1.sym
{
T 55600 47400 5 10 0 0 180 0 1
device=CAPACITOR
T 54900 47700 5 10 1 1 0 0 1
refdes=C7
T 55600 47200 5 10 0 0 180 0 1
symversion=0.1
T 55500 47700 5 10 1 1 0 0 1
value=100n
T 55800 48100 5 10 0 0 90 0 1
footprint=0805
}
N 54900 48400 54900 47900 4
N 57000 47900 57000 48400 4
N 57000 47900 55800 47900 4
C 55800 49900 1 0 0 switch-pb-1.sym
{
T 57000 50100 5 10 1 1 0 6 1
refdes=SW2
}
C 55800 50000 1 180 0 resistor-2.sym
{
T 55400 49650 5 10 0 0 180 0 1
device=RESISTOR
T 55000 50200 5 10 1 1 180 6 1
refdes=R8
T 55500 50200 5 10 1 1 180 6 1
value=1k
}
C 55800 49600 1 180 0 capacitor-1.sym
{
T 55600 48900 5 10 0 0 180 0 1
device=CAPACITOR
T 54900 49200 5 10 1 1 0 0 1
refdes=C5
T 55600 48700 5 10 0 0 180 0 1
symversion=0.1
T 55500 49200 5 10 1 1 0 0 1
value=100n
T 55800 49600 5 10 0 0 90 0 1
footprint=0805
}
N 54900 49900 54900 49400 4
N 57000 49400 57000 49900 4
N 57000 49400 55800 49400 4
C 55800 51500 1 0 0 switch-pb-1.sym
{
T 57000 51700 5 10 1 1 0 6 1
refdes=SW1
}
C 55800 51600 1 180 0 resistor-2.sym
{
T 55400 51250 5 10 0 0 180 0 1
device=RESISTOR
T 55000 51800 5 10 1 1 180 6 1
refdes=R7
T 55500 51800 5 10 1 1 180 6 1
value=1k
}
C 55800 51200 1 180 0 capacitor-1.sym
{
T 55600 50500 5 10 0 0 180 0 1
device=CAPACITOR
T 54900 50800 5 10 1 1 0 0 1
refdes=C4
T 55600 50300 5 10 0 0 180 0 1
symversion=0.1
T 55500 50800 5 10 1 1 0 0 1
value=100n
T 55800 51200 5 10 0 0 90 0 1
footprint=0805
}
N 54900 51500 54900 51000 4
N 57000 51000 57000 51500 4
N 57000 51000 55800 51000 4
N 57500 45400 57000 45400 4
N 57500 44700 57500 51500 4
N 57500 46900 57000 46900 4
N 57500 48400 57000 48400 4
N 57500 49900 57000 49900 4
N 57500 51500 57000 51500 4
N 54900 46900 54100 46900 4
N 54900 48400 54100 48400 4
N 54100 49900 54900 49900 4
N 54100 51500 54900 51500 4
N 48800 51100 49500 51100 4
C 49000 50200 1 90 0 capacitor-1.sym
{
T 48300 50400 5 10 0 0 90 0 1
device=CAPACITOR
T 48700 50400 5 10 1 1 0 6 1
refdes=C1
T 48100 50400 5 10 0 0 90 0 1
symversion=0.1
T 48700 50200 5 10 1 1 0 6 1
value=100n
T 49000 50200 5 10 0 0 0 0 1
footprint=0805
}
C 43400 50900 1 0 0 in-1.sym
{
T 43400 51200 5 10 0 0 0 0 1
device=INPUT
T 43400 51100 5 10 1 1 0 0 1
refdes=VCC_5V
}
C 43400 50000 1 0 0 in-1.sym
{
T 43400 50300 5 10 0 0 0 0 1
device=INPUT
T 43400 50200 5 10 1 1 0 0 1
refdes=GND
}
C 49500 51000 1 0 0 out-1.sym
{
T 49500 51300 5 10 0 0 0 0 1
device=OUTPUT
T 49500 51300 5 10 1 1 0 0 1
refdes=ADC_REF
}
C 43600 44800 1 0 1 out-1.sym
{
T 43600 45100 5 10 0 0 0 6 1
device=OUTPUT
T 45000 45100 5 10 1 1 0 6 1
refdes=ADC_POTI_BIT_ERROR_RATE
}
N 44000 51000 44800 51000 4
C 45500 49300 1 0 0 gnd-3.sym
{
T 45800 49450 5 10 1 1 0 5 1
net=GND:1
}
N 44000 50100 44500 50100 4
C 47100 44800 1 0 1 out-1.sym
{
T 47100 45100 5 10 0 0 0 6 1
device=OUTPUT
T 47900 45100 5 10 1 1 0 6 1
refdes=ADC_POTI_DROP_RATE
}
C 50100 44800 1 0 1 out-1.sym
{
T 50100 45100 5 10 0 0 0 6 1
device=OUTPUT
T 51700 45100 5 10 1 1 0 6 1
refdes=ADC_POTI_DROP_DURATION
}
C 54100 51400 1 0 1 out-1.sym
{
T 54100 51700 5 10 0 0 0 6 1
device=OUTPUT
T 54100 51700 5 10 1 1 0 6 1
refdes=SW_PWR
}
C 54100 49800 1 0 1 out-1.sym
{
T 54100 50100 5 10 0 0 0 6 1
device=OUTPUT
T 54100 50100 5 10 1 1 0 6 1
refdes=SW_ERROR_INHIBIT
}
C 54100 48300 1 0 1 out-1.sym
{
T 54100 48600 5 10 0 0 0 6 1
device=OUTPUT
T 54100 48600 5 10 1 1 0 6 1
refdes=SW_LO
}
C 54100 46800 1 0 1 out-1.sym
{
T 54100 47100 5 10 0 0 0 6 1
device=OUTPUT
T 54100 47100 5 10 1 1 0 6 1
refdes=SW_SOE
}
C 54100 44800 1 0 1 out-1.sym
{
T 54100 45100 5 10 0 0 0 6 1
device=OUTPUT
T 54100 45100 5 10 1 1 0 6 1
refdes=SW_SODS
}
N 43600 44900 44700 44900 4
N 54100 44900 54900 44900 4
C 44800 50100 1 0 0 tps731xx-1.sym
{
T 45000 51650 5 10 0 0 0 0 1
device=TPS731xx
T 45000 51450 5 10 1 1 0 0 1
refdes=U1
T 45000 52650 5 10 0 0 0 0 1
footprint=SOT25
T 45000 51250 5 10 1 1 0 0 1
value=TPS73125
}
C 47300 51100 1 0 0 generic-power.sym
{
T 47500 51350 5 10 1 1 0 3 1
net=VCC_2V5:1
}
N 44800 50600 44600 50600 4
N 44600 50600 44600 51000 4
C 44200 49700 1 0 0 gnd-3.sym
{
T 44500 49850 5 10 1 1 0 5 1
net=GND:1
}
C 47200 49700 1 90 0 capacitor-1.sym
{
T 46500 49900 5 10 0 0 90 0 1
device=CAPACITOR
T 47100 49900 5 10 1 1 0 0 1
refdes=C3
T 46300 49900 5 10 0 0 90 0 1
symversion=0.1
T 47100 49700 5 10 1 1 0 0 1
value=100n
T 47200 49700 5 10 0 0 0 0 1
footprint=0805
}
N 46800 50600 47000 50600 4
N 46800 51000 47500 51000 4
N 47500 51000 47500 51100 4
N 45800 49700 45800 50100 4
N 47000 49700 45800 49700 4