#!/usr/bin/env python3

import time
import gpiod
import re
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont
import sys

# For shared memory access
import mmap
import os
import struct

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

# --- Open Global Shared Memory ---
SHM_NAME = "/dev/shm/config_shared_memory"

SHM_FORMAT = "10i x 22d"  
SHM_SIZE = struct.calcsize(SHM_FORMAT)

shm_fd = os.open(SHM_NAME, os.O_RDWR)
actual_size = os.fstat(shm_fd).st_size
print(f"[DEBUG] Expected SHM_SIZE: {SHM_SIZE}, Actual size: {actual_size}")

if actual_size < SHM_SIZE:
    raise ValueError(f"Shared memory file is too small! Expected {SHM_SIZE}, but got {actual_size}")


shared_mem = mmap.mmap(shm_fd, SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
global shared_mem_values
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
  
class MenuEntry:
    def __init__(self, var_name, var_type, var_index, range_min=None, range_max=None):
        self.var_name = var_name
        self.var_type = var_type
        self.var_index = var_index
        self.range_min = range_min
        self.range_max = range_max
        self.showing_value = True  # Controls visibility
        self.editing_value = False  # Controls edit mode

    def __repr__(self):
        return f"MenuEntry(name={self.var_name}, type={self.var_type}, index={self.var_index}, range=({self.range_min}, {self.range_max}))"


# --- Menu Data ---
class MenuData:
    def __init__(self):
        self.menu_entries = []
        self.total_items = 0
        self.menu_index = 0
        self.last_index = -1
    def __repr__(self):
        return f"{self.menu_entries} menu index {self.menu_index}"
    
menu_data = MenuData()

def extract_struct_variables():
    """Extracts variable names, types, and their range (from comments) from the SharedConfig struct."""
    struct_pattern = r"struct SharedConfig\s*{([^}]*)};"  # Match the struct block
    var_pattern = r"\s*(int|double)\s+([a-zA-Z0-9_]+)\s*;.*?//\s*Range:\s*([\d\.-]+)\s*-\s*([\d\.-]+)"  # Match int/double + range

    try:
        with open(CONFIG_FILE, "r") as file:
            content = file.read()

            # Find struct block
            struct_match = re.search(struct_pattern, content, re.DOTALL)
            if not struct_match:
                print("Error: Could not find SharedConfig struct in config file.")
                sys.exit(1)

            struct_body = struct_match.group(1)  # Extract struct content

            # Extract variables from the struct and assign an index
            extracted_entries = []

            # Add "Close Menu" option
            close_entry = MenuEntry("Close Menu", "int", -1)
            close_entry.showing_value = False
            extracted_entries.append(close_entry)

            for idx, match in enumerate(re.finditer(var_pattern, struct_body)):
                var_type, var_name, range_min, range_max = match.groups()
                
                # Convert range values to appropriate type
                range_min = float(range_min) if '.' in range_min else int(range_min)
                range_max = float(range_max) if '.' in range_max else int(range_max)

                extracted_entries.append(MenuEntry(var_name, var_type, idx, range_min, range_max))

            return extracted_entries

    except FileNotFoundError:
        print(f"Error: {CONFIG_FILE} not found!")
        sys.exit(1)


'''
def extract_struct_variables():
    """Extracts variable names and types from the SharedConfig struct and assigns an index."""
    struct_pattern = r"struct SharedConfig\s*{([^}]*)};"  # Match the struct block
    var_pattern = r"\s*(int|double)\s+([a-zA-Z0-9_]+)\s*;"  # Match int/double variables

    try:
        with open(CONFIG_FILE, "r") as file:
            content = file.read()

            # Find struct block
            struct_match = re.search(struct_pattern, content, re.DOTALL)
            if not struct_match:
                print("Error: Could not find SharedConfig struct in config file.")
                sys.exit(1)

            struct_body = struct_match.group(1)  # Extract struct content

            # Extract variables from the struct and assign an index
            extracted_entries = []
            # Add close option
            close_entry = MenuEntry("Close Menu", "int", -1)
            close_entry.showing_value = False
            extracted_entries.append(close_entry)
            for idx, match in enumerate(re.finditer(var_pattern, struct_body)):
                var_type, var_name = match.groups()
                extracted_entries.append(MenuEntry(var_name, var_type, idx))

            return extracted_entries

    except FileNotFoundError:
        print(f"Error: {CONFIG_FILE} not found!")
        sys.exit(1)
'''

def draw_menu():
    global menu_data, shared_mem_values
                
    print(f"menu_data.menu_index: {menu_data.menu_index}")    
    # Ensure menu_index is within valid bounds
    #print(menu_data)
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
            #print(f"[DEBUG] Menu Entry: {entry}")
            y = 5 + y_offset * 14

            if i == menu_data.menu_index:
                draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                
                if(entry.var_name == "Close Menu"):
                    draw_obj.text((5, y), f"      {entry.var_name}", font=font_small, fill="black")

                else:
                    draw_obj.text((5, y), f"{i-1} - {entry.var_name}", font=font_small, fill="black")

                # Show value below name
                if entry.showing_value:
                    y_offset += 1
                    y = 5 + y_offset * 14
                    value_text = f"{shared_mem_values[entry.var_index]:.2f}"
                    if entry.editing_value:
                        draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                        draw_obj.text((20, y), value_text, font=font_small, fill="black")
                    else:
                        draw_obj.text((20, y), value_text, font=font_small, fill="white")
            
            else:
                if(entry.var_name == "Close Menu"):
                    draw_obj.text((5, y), f"      {entry.var_name}", font=font_small, fill="white")

                else:
                    draw_obj.text((5, y), f"{i-1} - {entry.var_name}", font=font_small, fill="white")

            y_offset += 1

# --- Handle Encoder Rotation ---
def handle_rotation_menu( value : float, increment : float):
    global clk, dt

    # Static variable for counting increments
    if not hasattr(handle_rotation_menu, "incrementCounter"):
        handle_rotation_menu.incrementCounter = 0
    
    if not hasattr(handle_rotation_menu, "increment"):
        handle_rotation_menu.increment = increment

    # Last accepted transition time
    if not hasattr(handle_rotation_menu, "last_rotary_state"):
        handle_rotation_menu.last_rotary_state = (clk.get_value() << 1) | dt.get_value()
        handle_rotation_menu.two_steps_ahead_state = handle_rotation_menu.last_rotary_state

    if not hasattr(handle_rotation_menu, "last_time"):
        handle_rotation_menu.last_time = 0

    current_time = time.monotonic()
    if current_time - handle_rotation_menu.last_time < DEBOUNCE_DELAY:
        # If we're within the debounce period, ignore this change.
        return value
    
    if current_time - handle_rotation_menu.last_time > INCREMENT_FAST_INCREASE_DELAY:
        handle_rotation_menu.increment = increment
        handle_rotation_menu.incrementCounter = 0
        
    
    if handle_rotation_menu.incrementCounter == 5 :
        handle_rotation_menu.increment = 1

    # Combine current clk and dt into a two-bit state.
    current_rotary_state = (clk.get_value() << 1) | dt.get_value()

    # If the state hasn’t changed, nothing to do.
    if ((current_rotary_state == handle_rotation_menu.last_rotary_state) or 
    (handle_rotation_menu.two_steps_ahead_state == current_rotary_state)):
        return value
    
    transition = (handle_rotation_menu.last_rotary_state, current_rotary_state)
    if transition in transition_table:
        step = transition_table[transition]
        value += (step * handle_rotation_menu.increment)
        handle_rotation_menu.incrementCounter += 1
        handle_rotation_menu.last_time = current_time  # Update the time after a valid transition.
        print(f"[DEBUG] {value}, Transition: {transition}, Step: {step}, "
              f"Counter: {handle_rotation_menu.incrementCounter}, Increment: {handle_rotation_menu.increment}")
    else:
        # The transition isn’t valid – it might be noise or an intermediate state.
        print(f"[DEBUG] Ignored invalid/spurious transition: {transition}")

    # Update the last state for the next call.
    handle_rotation_menu.two_steps_ahead_state = handle_rotation_menu.last_rotary_state
    handle_rotation_menu.last_rotary_state = current_rotary_state

    return value

# --- Handle Encoder Rotation ---
def handle_rotation_entry( selected_entry : MenuEntry, increment : float):
    global clk, dt, shared_mem_values

    # Static variable for counting increments
    if not hasattr(handle_rotation_entry, "incrementCounter"):
        handle_rotation_entry.incrementCounter = 0
    
    if not hasattr(handle_rotation_entry, "increment"):
        handle_rotation_entry.increment = increment

    # Last accepted transition time
    if not hasattr(handle_rotation_entry, "last_rotary_state"):
        handle_rotation_entry.last_rotary_state = (clk.get_value() << 1) | dt.get_value()
        handle_rotation_entry.two_steps_ahead_state = handle_rotation_entry.last_rotary_state

    if not hasattr(handle_rotation_entry, "last_time"):
        handle_rotation_entry.last_time = 0

    current_time = time.monotonic()
    if current_time - handle_rotation_entry.last_time < DEBOUNCE_DELAY:
        # If we're within the debounce period, ignore this change.
        return
    
    if current_time - handle_rotation_entry.last_time > INCREMENT_FAST_INCREASE_DELAY:
        handle_rotation_entry.increment = increment
        handle_rotation_entry.incrementCounter = 0
        
    
    if handle_rotation_entry.incrementCounter == 5 :
        handle_rotation_entry.increment = 1

    # Combine current clk and dt into a two-bit state.
    current_rotary_state = (clk.get_value() << 1) | dt.get_value()

    # If the state hasn’t changed, nothing to do.
    if ((current_rotary_state == handle_rotation_entry.last_rotary_state) or 
    (handle_rotation_entry.two_steps_ahead_state == current_rotary_state)):
        return
    
    transition = (handle_rotation_entry.last_rotary_state, current_rotary_state)
    if transition in transition_table:
        step = transition_table[transition]
        can_decrease = (-1 == step and shared_mem_values[selected_entry.var_index] > selected_entry.range_min)
        can_increase = (1 == step and shared_mem_values[selected_entry.var_index] < selected_entry.range_max)

        if can_decrease or can_increase:
            shared_mem_values[selected_entry.var_index] += (step * handle_rotation_entry.increment)
            handle_rotation_entry.incrementCounter += 1
            handle_rotation_entry.last_time = current_time  # Update time after a valid transition.

            print(f"[DEBUG] {shared_mem_values[selected_entry.var_index]}, Transition: {transition}, Step: {step}, "
                f"selected_entry.range_min: {selected_entry.range_min}, Increment: {handle_rotation_entry.increment}")

    else:
        # The transition isn’t valid – it might be noise or an intermediate state.
        print(f"[DEBUG] Ignored invalid/spurious transition: {transition}")

    # Update the last state for the next call.
    handle_rotation_entry.two_steps_ahead_state = handle_rotation_entry.last_rotary_state
    handle_rotation_entry.last_rotary_state = current_rotary_state

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
            print(f"[DEBUG] Saved Value: {shared_mem_values[selected_entry.var_index]}")
        elif selected_entry.showing_value:
            selected_entry.editing_value = True
            print("[DEBUG] Entering Value Edit Mode")
        else:
            selected_entry.showing_value = not selected_entry.showing_value
            print(f"[DEBUG] Toggle Value Display: {selected_entry.showing_value}")

        draw_menu()  # Ensure display updates when value is shown/hidden

# --- Main Loop ---
def main():
    global menu_data, shared_mem, shared_mem_values

    last_variable_value = 0

    menu_data.menu_entries = extract_struct_variables()
    menu_data.total_items = len(menu_data.menu_entries)
    print(f"menu_data.total_items: {menu_data.total_items}")


    
    shared_mem.seek(0)
    shared_mem_values = struct.unpack(SHM_FORMAT, shared_mem.read(SHM_SIZE))            
    shared_mem_values = list(shared_mem_values)
    
    #print(f"[DEBUG] Shared memory values: {shared_mem_values}")
    #print(f"[DEBUG] Expected number of values: {len(shared_mem_values)}")

    print(f"menu_data.menu_index: {menu_data.menu_index}")
    draw_menu()

    try:
        while True:

            shared_mem.seek(0)
            shared_mem_values = struct.unpack(SHM_FORMAT, shared_mem.read(SHM_SIZE))
            shared_mem_values = list(shared_mem_values)


            handle_button()
            selected_entry = menu_data.menu_entries[menu_data.menu_index]
            if selected_entry.editing_value:
                if selected_entry.var_name == "Close Menu":
                    break
                if selected_entry.var_type == "double":
                    handle_rotation_entry(selected_entry, 0.1)

                if selected_entry.var_type == "int":
                    handle_rotation_entry(selected_entry, 1)

                shared_mem[:SHM_SIZE] = struct.pack(SHM_FORMAT, *shared_mem_values)

                if shared_mem_values[selected_entry.var_index] != last_variable_value:
                    last_variable_value = shared_mem_values[selected_entry.var_index]
                    draw_menu()
            else:
                menu_data.menu_index = handle_rotation_menu(menu_data.menu_index, 1)

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
