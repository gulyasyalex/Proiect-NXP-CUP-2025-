#!/usr/bin/env python3

import time
import gpiod
import re
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont
import sys

# --- Configuration ---
CONFIG_FILE = "include/config.h"

# I2C Configuration
I2C_BUS = 5
I2C_ADDRESS = 0x3C

# Rotary Encoder GPIO Configuration
GPIO_CHIP = 'gpiochip0'
CLK_LINE = 11
DT_LINE = 13
SW_LINE = 12

# Menu Settings
VISIBLE_ITEMS = 3
DEBOUNCE_TIME = 0.01


# --- Initialize I2C OLED Display ---
try:
    serial = i2c(port=I2C_BUS, address=I2C_ADDRESS)
    device = sh1106(serial, rotate=0)
except Exception as e:
    print(f"Failed to initialize OLED display: {e}")
    sys.exit(1)

# --- Fonts ---
font_small = ImageFont.load_default()
try:
    font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
except Exception as e:
    font_large = font_small


# --- Initialize GPIO ---
chip = gpiod.Chip(GPIO_CHIP)
clk = chip.get_line(CLK_LINE)
dt = chip.get_line(DT_LINE)
sw = chip.get_line(SW_LINE)

try:
    clk.request(consumer="menu_rotary", type=gpiod.LINE_REQ_EV_RISING_EDGE | gpiod.LINE_REQ_EV_FALLING_EDGE)
    dt.request(consumer="menu_rotary", type=gpiod.LINE_REQ_EV_RISING_EDGE | gpiod.LINE_REQ_EV_FALLING_EDGE)
    sw.request(consumer="menu_rotary", type=gpiod.LINE_REQ_EV_FALLING_EDGE)
except Exception as e:
    print(f"Failed to request GPIO lines: {e}")
    sys.exit(1)

# --- Handle Rotation ---
DEBOUNCE_DELAY = 0.008  # Debounce delay in seconds (5 ms)
INCREMENT_FAST_INCREASE_DELAY = 0.3 # Debounce delay in seconds (5 ms)
# Initial state (two-bit value, 0: 00, 1: 01, 2: 10, 3: 11)
# Lookup table for valid transitions:
# For a typical rotary encoder, if the sequence goes:
#   00 -> 01 -> 11 -> 10 -> 00, it is one direction (say, clockwise)
#   00 -> 10 -> 11 -> 01 -> 00, it is the other (counterclockwise)
# Here, we assign a step value for a valid transition.
# States: 0 = 00, 1 = 01, 2 = 10, 3 = 11
transition_table = {
    (0, 1): -1,  # 00 -> 01 : clockwise step
    (1, 3): -1,  # 01 -> 11 : clockwise step
    (3, 2): -1,  # 11 -> 10 : clockwise step
    (2, 0): -1,  # 10 -> 00 : clockwise step
    (0, 2): 1, # 00 -> 10 : counterclockwise step
    (2, 3): 1, # 10 -> 11 : counterclockwise step
    (3, 1): 1, # 11 -> 01 : counterclockwise step
    (1, 0): 1, # 01 -> 00 : counterclockwise step
}

# --- Menu Entry ---
class MenuEntry:
    def __init__(self, var_name, var_value):
        self.var_name = var_name
        self.var_value = var_value
        self.showing_value = True  # Controls visibility
        self.editing_value = False  # Controls edit mode

# --- Menu Data ---
class MenuData:
    def __init__(self):
        self.menu_entries = []
        self.total_items = 0
        self.menu_index = 0
        self.last_index = -1

menu_data = MenuData()

# --- Parse config.h and Extract Variables ---
def parse_config():
    extracted_entries = []
    pattern = r"^(double|int)\s+([a-zA-Z0-9_]+)\s*=\s*([\d\.\-]+);"

    try:
        with open(CONFIG_FILE, "r") as file:
            for line in file:
                match = re.match(pattern, line.strip())
                if match:
                    var_type, var_name, var_value = match.groups()
                    extracted_entries.append(MenuEntry(var_name, float(var_value)))
    except FileNotFoundError:
        print(f"Error: {CONFIG_FILE} not found!")
        sys.exit(1)

    return extracted_entries

def draw_menu():
    global menu_data

                    
    # Ensure menu_index is within valid bounds
    if menu_data.menu_index < 0:
        menu_data.menu_index = menu_data.total_items - 1
    if menu_data.menu_index >= menu_data.total_items:
        menu_data.menu_index = 0
        
    # Calculate the visible range dynamically
    start_index = max(0, menu_data.menu_index - (menu_data.menu_index % VISIBLE_ITEMS))
    end_index = min(start_index + VISIBLE_ITEMS, menu_data.total_items)

    print(f"[DEBUG] Drawing menu - Index: {menu_data.menu_index}, Showing Value: {menu_data.menu_entries[menu_data.menu_index].showing_value}, Editing: {menu_data.menu_entries[menu_data.menu_index].editing_value}, Visible Range: {start_index}-{end_index}")

    with canvas(device) as draw_obj:
        y_offset = 0
        for i in range(start_index, end_index):
            entry = menu_data.menu_entries[i]
            y = 5 + y_offset * 14

            if i == menu_data.menu_index:
                draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                draw_obj.text((5, y), f"{i} - {entry.var_name}", font=font_small, fill="black")

                # Show value below name
                if entry.showing_value:
                    y_offset += 1
                    y = 5 + y_offset * 14
                    value_text = f"{entry.var_value:.2f}"
                    if entry.editing_value:
                        draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                        draw_obj.text((20, y), value_text, font=font_small, fill="black")
                    else:
                        draw_obj.text((20, y), value_text, font=font_small, fill="white")
            
            else:
                draw_obj.text((5, y), f"{i} - {entry.var_name}", font=font_small, fill="white")

            y_offset += 1


def handle_rotation( value : float, increment : float):
    global clk, dt

    # Static variable for counting increments
    if not hasattr(handle_rotation, "incrementCounter"):
        handle_rotation.incrementCounter = 0
    
    if not hasattr(handle_rotation, "increment"):
        handle_rotation.increment = increment

    # Last accepted transition time
    if not hasattr(handle_rotation, "last_rotary_state"):
        handle_rotation.last_rotary_state = (clk.get_value() << 1) | dt.get_value()
        handle_rotation.two_steps_ahead_state = handle_rotation.last_rotary_state

    if not hasattr(handle_rotation, "last_time"):
        handle_rotation.last_time = 0

    current_time = time.monotonic()
    if current_time - handle_rotation.last_time < DEBOUNCE_DELAY:
        # If we're within the debounce period, ignore this change.
        return value
    
    if current_time - handle_rotation.last_time > INCREMENT_FAST_INCREASE_DELAY:
        handle_rotation.increment = increment
        handle_rotation.incrementCounter = 0
        
    
    if handle_rotation.incrementCounter == 5 :
        handle_rotation.increment = 1

    # Combine current clk and dt into a two-bit state.
    current_rotary_state = (clk.get_value() << 1) | dt.get_value()

    # If the state hasn’t changed, nothing to do.
    if ((current_rotary_state == handle_rotation.last_rotary_state) or 
    (handle_rotation.two_steps_ahead_state == current_rotary_state)):
        return value
    
    transition = (handle_rotation.last_rotary_state, current_rotary_state)
    if transition in transition_table:
        step = transition_table[transition]
        value += (step * handle_rotation.increment)
        handle_rotation.incrementCounter += 1
        handle_rotation.last_time = current_time  # Update the time after a valid transition.
        print(f"[DEBUG] {value}, Transition: {transition}, Step: {step}, "
              f"Counter: {handle_rotation.incrementCounter}, Increment: {handle_rotation.increment}")
    else:
        # The transition isn’t valid – it might be noise or an intermediate state.
        print(f"[DEBUG] Ignored invalid/spurious transition: {transition}")

    # Update the last state for the next call.
    handle_rotation.two_steps_ahead_state = handle_rotation.last_rotary_state
    handle_rotation.last_rotary_state = current_rotary_state

    return value


# --- Handle Button Press ---
def handle_button():
    global menu_data

    if sw.get_value() == 0:
        time.sleep(0.2)
        while sw.get_value() == 0:
            time.sleep(DEBOUNCE_TIME)

        selected_entry = menu_data.menu_entries[menu_data.menu_index]

        if selected_entry.editing_value:
            selected_entry.editing_value = False  # Save the new value
            print(f"[DEBUG] Saved Value: {selected_entry.var_value}")
        elif selected_entry.showing_value:
            selected_entry.editing_value = True
            print("[DEBUG] Entering Value Edit Mode")
        else:
            selected_entry.showing_value = not selected_entry.showing_value
            print(f"[DEBUG] Toggle Value Display: {selected_entry.showing_value}")

        draw_menu()  # Ensure display updates when value is shown/hidden

# --- Main Loop ---
def main():
    global menu_data

    last_variable_value = 0
    
    menu_data.menu_entries = parse_config()
    menu_data.total_items = len(menu_data.menu_entries)

    draw_menu()
    print("Rotary Encoder + Menu Example (Python + Luma.OLED + libgpiod)")

    try:
        while True:

            handle_button()
            selected_entry = menu_data.menu_entries[menu_data.menu_index]
            if selected_entry.editing_value:
                selected_entry.var_value = handle_rotation(selected_entry.var_value, 0.1)

                if selected_entry.var_value != last_variable_value:
                    last_variable_value = selected_entry.var_value
                    draw_menu()
            else:
                menu_data.menu_index = handle_rotation(menu_data.menu_index, 1)

                if menu_data.menu_index != menu_data.last_index:
                    menu_data.last_index = menu_data.menu_index
                    draw_menu()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        chip.close()

if __name__ == "__main__":
    main()
