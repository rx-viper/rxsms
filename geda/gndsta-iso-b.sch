v 20110115 2
T 28700 50400 2 20 1 0 0 0 1
only +/- 50V Isolation
C 22500 40000 0 0 0 title-A3.sym
C 28500 48100 1 0 0 in-1.sym
{
T 28500 48400 5 10 0 0 0 0 1
device=INPUT
T 28100 48100 5 10 1 1 0 0 1
refdes=T1IN
}
C 28300 47900 1 0 0 in-1.sym
{
T 28300 48200 5 10 0 0 0 0 1
device=INPUT
T 27900 47900 5 10 1 1 0 0 1
refdes=T2IN
}
C 29100 47700 1 0 1 out-1.sym
{
T 29100 48000 5 10 0 0 0 6 1
device=OUTPUT
T 28500 47700 5 10 1 1 0 6 1
refdes=R1OUT
}
C 28900 47500 1 0 1 out-1.sym
{
T 28900 47800 5 10 0 0 0 6 1
device=OUTPUT
T 28300 47500 5 10 1 1 0 6 1
refdes=R2OUT
}
N 28900 48000 29100 48000 4
N 28900 47600 29100 47600 4
C 28600 44300 1 270 1 capacitor-1.sym
{
T 29300 44500 5 10 0 0 90 2 1
device=CAPACITOR
T 28700 44500 5 10 1 1 0 6 1
refdes=C3
T 29500 44500 5 10 0 0 90 2 1
symversion=0.1
T 28700 44300 5 10 1 1 0 6 1
value=470n
T 28600 44300 5 10 0 0 0 6 1
footprint=0805
}
C 28600 45400 1 270 1 capacitor-1.sym
{
T 29300 45600 5 10 0 0 90 2 1
device=CAPACITOR
T 28700 45600 5 10 1 1 0 6 1
refdes=C2
T 29500 45600 5 10 0 0 90 2 1
symversion=0.1
T 28700 45400 5 10 1 1 0 6 1
value=470n
T 28600 45400 5 10 0 0 0 6 1
footprint=0805
}
C 32000 44300 1 270 1 capacitor-1.sym
{
T 32700 44500 5 10 0 0 90 2 1
device=CAPACITOR
T 32300 44500 5 10 1 1 0 0 1
refdes=C6
T 32900 44500 5 10 0 0 90 2 1
symversion=0.1
T 32300 44300 5 10 1 1 0 0 1
value=100n
T 32000 44300 5 10 0 0 0 6 1
footprint=0805
}
C 32000 45400 1 270 1 capacitor-1.sym
{
T 32700 45600 5 10 0 0 90 2 1
device=CAPACITOR
T 32300 45600 5 10 1 1 0 0 1
refdes=C4
T 32900 45600 5 10 0 0 90 2 1
symversion=0.1
T 32300 45400 5 10 1 1 0 0 1
value=470n
T 32000 45400 5 10 0 0 0 6 1
footprint=0805
}
C 27300 45400 1 270 1 capacitor-1.sym
{
T 28000 45600 5 10 0 0 90 2 1
device=CAPACITOR
T 27400 45600 5 10 1 1 0 6 1
refdes=C1
T 28200 45600 5 10 0 0 90 2 1
symversion=0.1
T 27400 45400 5 10 1 1 0 6 1
value=1u
T 27300 45400 5 10 0 0 0 6 1
footprint=0805
}
C 34500 44700 1 270 1 capacitor-1.sym
{
T 35200 44900 5 10 0 0 90 2 1
device=CAPACITOR
T 34800 44900 5 10 1 1 0 0 1
refdes=C9
T 35400 44900 5 10 0 0 90 2 1
symversion=0.1
T 34800 44700 5 10 1 1 0 0 1
value=2u2
T 34500 44700 5 10 0 0 0 6 1
footprint=0805
}
C 31000 43200 1 0 1 capacitor-1.sym
{
T 30800 43900 5 10 0 0 180 2 1
device=CAPACITOR
T 30100 43200 5 10 1 1 0 0 1
refdes=C5
T 30800 44100 5 10 0 0 180 2 1
symversion=0.1
T 30100 43000 5 10 1 1 0 0 1
value=10n/100V
T 31000 43200 5 10 0 0 90 6 1
footprint=0805
}
C 29100 43800 1 0 0 MAX3250-1.sym
{
T 31600 48550 5 10 1 1 0 6 1
device=MAX3250
T 29400 48550 5 10 1 1 0 0 1
refdes=U1
T 29400 49850 5 10 0 0 0 0 1
footprint=SSOP28
}
N 28800 46300 29100 46300 4
N 29100 46300 29100 46200 4
N 29100 45400 28800 45400 4
N 28800 45200 29100 45200 4
N 28800 44300 29100 44300 4
N 29100 44300 29100 44400 4
N 32200 44300 31900 44300 4
N 31900 44300 31900 44400 4
N 31900 45200 32200 45200 4
N 31900 45400 32200 45400 4
N 32200 46300 31900 46300 4
N 31900 46300 31900 46200 4
C 32900 44700 1 270 1 capacitor-1.sym
{
T 33600 44900 5 10 0 0 90 2 1
device=CAPACITOR
T 33200 44900 5 10 1 1 0 0 1
refdes=C7
T 33800 44900 5 10 0 0 90 2 1
symversion=0.1
T 33200 44700 5 10 1 1 0 0 1
value=470n
T 32900 44700 5 10 0 0 0 6 1
footprint=0805
}
C 27400 48600 1 270 0 in-1.sym
{
T 27700 48600 5 10 0 0 270 0 1
device=INPUT
T 27200 48700 5 10 1 1 0 0 1
refdes=VCC_3V3
}
N 27500 47200 29100 47200 4
N 27500 46300 27500 48000 4
N 29100 46600 28400 46600 4
N 28400 46600 28400 47200 4
C 25500 43900 1 0 0 in-1.sym
{
T 25500 44200 5 10 0 0 0 0 1
device=INPUT
T 25100 43900 5 10 1 1 0 0 1
refdes=GND
}
N 26100 44000 29100 44000 4
N 27500 45400 27500 44000 4
N 30100 43400 28800 43400 4
N 28800 43400 28800 44000 4
N 31000 43400 32400 43400 4
N 32400 43400 32400 44000 4
N 31900 44000 35100 44000 4
C 35700 43900 1 0 1 in-1.sym
{
T 35700 44200 5 10 0 0 0 6 1
device=INPUT
T 36600 43900 5 10 1 1 0 6 1
refdes=GND_EXT
}
C 32500 47700 1 0 1 in-1.sym
{
T 32500 48000 5 10 0 0 0 6 1
device=INPUT
T 33000 47700 5 10 1 1 0 6 1
refdes=R1IN
}
C 31900 48100 1 0 0 out-1.sym
{
T 31900 48400 5 10 0 0 0 0 1
device=OUTPUT
T 32600 48100 5 10 1 1 0 0 1
refdes=T1OUT
}
C 32100 47900 1 0 0 out-1.sym
{
T 32100 48200 5 10 0 0 0 0 1
device=OUTPUT
T 32800 47900 5 10 1 1 0 0 1
refdes=T2OUT
}
C 32700 47500 1 0 1 in-1.sym
{
T 32700 47800 5 10 0 0 0 6 1
device=INPUT
T 33200 47500 5 10 1 1 0 6 1
refdes=R2IN
}
N 32100 48000 31900 48000 4
N 32100 47600 31900 47600 4
C 33700 44700 1 270 1 capacitor-1.sym
{
T 34400 44900 5 10 0 0 90 2 1
device=CAPACITOR
T 34000 44900 5 10 1 1 0 0 1
refdes=C8
T 34600 44900 5 10 0 0 90 2 1
symversion=0.1
T 34000 44700 5 10 1 1 0 0 1
value=470n
T 33700 44700 5 10 0 0 0 6 1
footprint=0805
}
N 31900 46600 33100 46600 4
N 33100 46600 33100 45600 4
N 31900 46800 33900 46800 4
N 33900 46800 33900 45600 4
N 31900 47200 34700 47200 4
N 34700 47200 34700 45600 4
N 34700 44700 34700 44000 4
N 33900 44700 33900 44000 4
N 33100 44700 33100 44000 4
C 24900 50300 1 0 0 in-1.sym
{
T 24900 50600 5 10 0 0 0 0 1
device=INPUT
T 24900 50600 5 10 1 1 0 0 1
refdes=VCC_5V
}
T 25800 50400 9 10 1 0 0 0 1
not used