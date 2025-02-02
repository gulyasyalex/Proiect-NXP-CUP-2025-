#!/usr/bin/env python3

from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont

def main():
    # 1. Specify the i2c port and address
    #    If your Debix board enumerates the bus as port=5, address=0x3C:
    serial = i2c(port=5, address=0x3C)

    # 2. Create the SH1106 device
    device = sh1106(serial)

    # 3. Draw something
    #    'canvas' gives us a PIL drawing canvas for the current device
    font = ImageFont.load_default()  # or load a TTF file
    with canvas(device) as draw:
        draw.text((0, 0), "Hello Debix!", font=font, fill="white")
        draw.text((0, 16), "This is SH1106", font=font, fill="white")

    print("Drew text on SH1106 display. Press Ctrl+C to exit.")
    # Keep running so you can see the text
    while True:
        pass

if __name__ == "__main__":
    main()
