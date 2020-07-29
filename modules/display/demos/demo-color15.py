# color15.py Test/demo program for Adafruit ssd1351-based OLED displays
# Adafruit 1.5" 128*128 OLED display: https://www.adafruit.com/product/1431
# Adafruit 1.27" 128*96 display https://www.adafruit.com/product/1673
# For wiring details see drivers/ADAFRUIT.md in this repo.

# The MIT License (MIT)

# Copyright (c) 2018 Peter Hinch

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

height = 96  # 1.27 inch 96*128 (rows*cols) display
# height = 128 # 1.5 inch 128*128 display

from machine import Pin,SPI
import gc,sys
from ST7735 import Display,color565
import cmath
import utime
import uos
from writer import Writer, CWriter
from nanogui import Label, Meter, LED, Dial, Pointer, refresh

if sys.platform == 'esp8266':
    print('1.4 inch TFT screen test on ESP8266')
    SPI_CS = 16
    SPI_DC = 15
    spi = SPI(1)
    
elif sys.platform == 'esp32':
    print('1.4 inch TFT screen test on ESP32')
    sck = Pin(18)
    miso= Pin(19)
    mosi= Pin(23)
    SPI_CS = 26
    SPI_DC = 5
    spi = SPI(2, baudrate=32000000, sck=sck, mosi=mosi, miso=miso)

st7735 =Display(spi,SPI_CS,SPI_DC)    
# Fonts
import fonts.arial10 as arial10
import fonts.freesans20 as freesans20

GREEN = color565(0, 255, 0)
RED = color565(255, 0, 0)
BLUE = color565(0, 0, 255)
YELLOW = color565(255, 255, 0)
BLACK = 0
CWriter.set_textpos(st7735, 0, 0)  # In case previous tests have altered it
wri = CWriter(st7735, arial10, GREEN, BLACK, verbose=False)
wri.set_clip(True, True, False)

def meter():
    print('Meter test.')
    refresh(st7735, True)  # Clear any prior image
    color = lambda v : RED if v > 0.7 else YELLOW if v > 0.5 else GREEN
    txt = lambda v : 'ovr' if v > 0.7 else 'high' if v > 0.5 else 'ok'
    m0 = Meter(wri, 5, 2, divisions = 4, ptcolor=YELLOW,
              label='left', style=Meter.BAR, legends=('0.0', '0.5', '1.0'))
    l0 = LED(wri, st7735.height - 16 - wri.height, 2, bdcolor=YELLOW, label ='over')
    m1 = Meter(wri, 5, 50, divisions = 4, ptcolor=YELLOW,
              label='right', style=Meter.BAR, legends=('0.0', '0.5', '1.0'))
    l1 = LED(wri, st7735.height - 16 - wri.height, 50, bdcolor=YELLOW, label ='over')
    m2 = Meter(wri, 5, 98, divisions = 4, ptcolor=YELLOW,
              label='bass', style=Meter.BAR, legends=('0.0', '0.5', '1.0'))
    l2 = LED(wri, st7735.height - 16 - wri.height, 98, bdcolor=YELLOW, label ='over')
    steps = 10
    for n in range(steps):
        v = int.from_bytes(uos.urandom(3),'little')/16777216
        m0.value(v, color(v))
        l0.color(color(v))
        l0.text(txt(v), fgcolor=color(v))
        v = n/steps
        m1.value(v, color(v))
        l1.color(color(v))
        l1.text(txt(v), fgcolor=color(v))
        v = 1 - n/steps
        m2.value(v, color(v))
        l2.color(color(v))
        l2.text(txt(v), fgcolor=color(v))
        refresh(st7735)
        utime.sleep(1)


def multi_fields(t):
    print('Dynamic labels.')
    refresh(st7735, True)  # Clear any prior image
    nfields = []
    dy = wri.height + 6
    y = 2
    col = 15
    width = wri.stringlen('99.99')
    for txt in ('X:', 'Y:', 'Z:'):
        Label(wri, y, 0, txt)  # Use wri default colors
        nfields.append(Label(wri, y, col, width, bdcolor=None))  # Specify a border, color TBD
        y += dy

    end = utime.ticks_add(utime.ticks_ms(), t * 1000)
    while utime.ticks_diff(end, utime.ticks_ms()) > 0:
        for field in nfields:
            value = int.from_bytes(uos.urandom(3),'little')/167772
            overrange =  None if value < 70 else YELLOW if value < 90 else RED
            field.value('{:5.2f}'.format(value), fgcolor = overrange, bdcolor = overrange)
        refresh(st7735)
        utime.sleep(1)
    Label(wri, 0, 64, ' OK ', True, fgcolor = RED)
    refresh(st7735)
    utime.sleep(1)

def vari_fields():
    print('Variable label styles.')
    refresh(st7735, True)  # Clear any prior image
    wri_large = CWriter(st7735, freesans20, GREEN, BLACK, verbose=False)
    wri_large.set_clip(True, True, False)
    Label(wri_large, 0, 0, 'Text')
    Label(wri_large, 20, 0, 'Border')
    width = wri_large.stringlen('Yellow')
    lbl_text = Label(wri_large, 0, 65, width)
    lbl_bord = Label(wri_large, 20, 65, width)
    lbl_text.value('Red')
    lbl_bord.value('Red')
    lbl_var = Label(wri_large, 50, 2, '25.46', fgcolor=RED, bdcolor=RED)
    refresh(st7735)
    utime.sleep(2)
    lbl_text.value('Red')
    lbl_bord.value('Yellow')
    lbl_var.value(bdcolor=YELLOW)
    refresh(st7735)
    utime.sleep(2)
    lbl_text.value('Red')
    lbl_bord.value('None')
    lbl_var.value(bdcolor=False)
    refresh(st7735)
    utime.sleep(2)
    lbl_text.value('Yellow')
    lbl_bord.value('None')
    lbl_var.value(fgcolor=YELLOW)
    refresh(st7735)
    utime.sleep(2)
    lbl_text.value('Blue')
    lbl_bord.value('Green')
    lbl_var.value('18.99', fgcolor=BLUE, bdcolor=GREEN)
    Label(wri, st7735.height - wri.height - 2, 0, 'Done', fgcolor=RED)
    refresh(st7735)

def clock(x):
    print('Clock test.')
    refresh(st7735, True)  # Clear any prior image
    lbl = Label(wri, 5, 85, 'Clock')
    dial = Dial(wri, 5, 5, height = 75, ticks = 12, bdcolor=None, label=50)  # Border in fg color
    hrs = Pointer(dial)
    mins = Pointer(dial)
    hrs.value(0 + 0.7j, RED)
    mins.value(0 + 0.9j, YELLOW)
    dm = cmath.rect(1, -cmath.pi/30)  # Rotate by 1 minute (CW)
    dh = cmath.rect(1, -cmath.pi/1800)  # Rotate hours by 1 minute
    for n in range(x):
        refresh(st7735)
        utime.sleep_ms(200)
        mins.value(mins.value() * dm, YELLOW)
        hrs.value(hrs.value() * dh, RED)
        dial.text('ticks: {}'.format(n))
    lbl.value('Done')

def compass(x):
    print('Compass test.')
    refresh(st7735, True)  # Clear any prior image
    dial = Dial(wri, 5, 5, height = 75, bdcolor=None, label=50, style = Dial.COMPASS)
    bearing = Pointer(dial)
    bearing.value(0 + 1j, RED)
    dh = cmath.rect(1, -cmath.pi/30)  # Rotate by 6 degrees CW
    for n in range(x):
        utime.sleep_ms(200)
        bearing.value(bearing.value() * dh, RED)
        refresh(st7735)

print('Color display test is running.')
clock(70)
compass(70)
meter()
multi_fields(t = 10)
vari_fields()
print('Test complete.')
