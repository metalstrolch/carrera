# CarAttiny25 metalstrolch edition
Custom firmware for Carrera Digital 143 decoder.
Adds most missing features compared to stock digital 132 decoder.
Most of the added features are available on 143 red box as well using controller click programming.
The programming is compatible to the controller click programming of the carrera 132 blackbox.
The CU does nothing else than automate the click programming.

Additional features to stock 143 firmware:
- Ghost car funciton,
- Pace car function with CU and pit lane,
- Speed selection with CU as usual, or with click code on Redbox / Blackbox,
- Front light support with addition hardware
- Maximum possible speed. Stock firmware limits to around 80% possible speed.

I also plan to add support for the stop light.

Things yet to be done or imposible to do:
- There is no hardware support for brakes, so no brake strength selection.
- Real mode fuel simulation still missing. There is no protocol description how the speed degrade on full tank is done. Maybe later.
- Old BB car centric fuel programming. This is obsolete by carrera so I won't invest on that albeit possible.


# MiniCarreraDecoder
Reduced copy of the original decoder Carrera Digital 143.

**v4_SingleLayer** - is a single sided PCB, convenient for home production.

**v6** - a double sided variant, has a smaller size.

<img src="https://github.com/azya52/carrera/blob/master/Images/v6_scheme.png" width="70%">

* U1 - ATtiny25V-10SU (or ATtiny25-20SU)
* U3 - IRLML2030TRPBF
* R1 - 0603 680Ω
* R2 - 0603 390Ω
* D1 - FDLL300A
* D2 - LL4148
* D3 - BZX384-C5V6.115
* D4 - BAS321.115
* C1 - 0603 100nF
* C2 - 0.1mF (electrolitic)
* C3 - 0603 100nF
* C4 - 1μF (tantalum)

<img src="https://github.com/azya52/carrera/blob/master/Images/v6_top.png" width="30%"><img src="https://github.com/azya52/carrera/blob/master/Images/v6_bottom.png" width="30%">

<img src="https://github.com/azya52/carrera/blob/master/Images/example_0.jpg">
