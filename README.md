This is really old code and has lots of old bugs.

This uses the SPI hardware at Clk/2 to generate a 125 ns dotclock
video output.  This works out to about 400 horizontal lines and
with my standard 5x7 font for the character display it can do a
45x20 display.

The NTSC voltages are created with a two resistors (330 Ohm on sync
(PORTB7), 1 K Ohm on data (PORTB2, MOSI)).

