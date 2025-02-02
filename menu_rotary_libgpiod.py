#!/usr/bin/env python3
'''
import time
import gpiod
import re
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont
import sys

# --- Configuration ---

CONFIG_FILE = "include/config.h"  # Path to config.h

# I2C Configuration
I2C_BUS = 5           # Corresponds to /dev/i2c-5
I2C_ADDRESS = 0x3C    # Detected via i2cdetect

# Rotary Encoder GPIO Configuration
GPIO_CHIP = 'gpiochip0'  # Replace with your GPIO chip
CLK_LINE = 11            # GPIO1_IO11
DT_LINE = 13             # GPIO1_IO13
SW_LINE = 12             # GPIO1_IO12

# Menu Settings
VISIBLE_ITEMS = 4
DEBOUNCE_TIME = 0.01  # 10ms debounce

# --- Initialize I2C OLED Display ---

try:
    serial = i2c(port=I2C_BUS, address=I2C_ADDRESS)
    device = sh1106(serial, rotate=0)
except Exception as e:
    print(f"Failed to initialize OLED display: {e}")
    sys.exit(1)

class MenuEntry:
    def __init__(self, var_name, var_value):
        self.var_name = var_name
        self.var_value = var_value

# --- Initialize GPIO Lines ---

# Open the GPIO chip
chip = gpiod.Chip(GPIO_CHIP)

# Request lines
clk = chip.get_line(CLK_LINE)
dt = chip.get_line(DT_LINE)
sw = chip.get_line(SW_LINE)

if not clk or not dt or not sw:
    print("Failed to get GPIO lines. Check configuration.")
    sys.exit(1)

# Configure lines: CLK and DT as input, SW as input with pull-up
try:
    clk.request(consumer="menu_rotary", type=gpiod.LINE_REQ_EV_RISING_EDGE | gpiod.LINE_REQ_EV_FALLING_EDGE)
    dt.request(consumer="menu_rotary", type=gpiod.LINE_REQ_EV_RISING_EDGE | gpiod.LINE_REQ_EV_FALLING_EDGE)
    sw.request(consumer="menu_rotary", type=gpiod.LINE_REQ_EV_FALLING_EDGE)  
except Exception as e:
    print(f"Failed to request GPIO lines: {e}")
    sys.exit(1)

# --- Parse config.h and Extract Variables ---
def parse_config():
    extracted_entries = []
    pattern = r"^(double|int)\s+([a-zA-Z0-9_]+)\s*=\s*([\d\.\-]+);"  # Regex to match `double` or `int` variables

    try:
        with open(CONFIG_FILE, "r") as file:
            for line in file:
                match = re.match(pattern, line.strip())
                if match:
                    var_type, var_name, var_value = match.groups()
                    extracted_entries.append(MenuEntry(var_name, var_value))  # Format as "variable: value"
    except FileNotFoundError:
        print(f"Error: {CONFIG_FILE} not found!")
        sys.exit(1)

    extracted_entries.append(MenuEntry("Return", 0))

    return extracted_entries

# --- Menu Data ---

menu_entries = parse_config()
if not menu_entries:
    menu_entries = ["No variables found in config.h"]

menu_index = 0       # Current highlighted item
last_index = -1      # For redraw checks
enter_menu = False   # True if button pressed

# --- Fonts ---

font_small = ImageFont.load_default()
try:
    font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    font_medium = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
except IOError:
    font_large = font_small  # Fallback if TTF not found

# --- Helper Functions ---

def draw_menu():
    global menu_index
    total_items = len(menu_entries) - 1 # -1 for return

    # Bounds checking
    if menu_index < 0:
        menu_index = total_items - 1
    if menu_index >= total_items:
        menu_index = 0

    # Calculate the visible window
    start_index = menu_index - (menu_index % VISIBLE_ITEMS)
    end_index = start_index + VISIBLE_ITEMS
    if end_index > total_items:
        start_index = max(total_items - VISIBLE_ITEMS, 0)
        end_index = total_items

    with canvas(device) as draw_obj:
        y_offset = 0
        for i in range(start_index, end_index):
            entry = menu_entries[i]

            y = 5 + y_offset * 14
            if i == menu_index:
                # Highlight
                draw_obj.rectangle((0, y-2, device.width, y+12), fill="white")
                draw_obj.text((5, y), f"{entry.var_name}", font=font_small, fill="black")
            else:
                draw_obj.text((5, y), f"{entry.var_name}", font=font_small, fill="white")
            y_offset += 1

def menu_selection_action():
    """
    Handles inline value display and editing without submenus.
    - Press switch on name → Show/Hide value
    - Press switch on value → Edit value
    - Rotate to change value while editing
    - Press switch again to save value and exit editing mode
    """
    global enter_menu, menu_index

    selected_entry = menu_entries[menu_index]
    showing_value = False  # Tracks if the value is visible
    modifying_value = False  # Tracks if we are modifying the value
    temp_value = float(selected_entry.var_value)  # Editable value

    while True:
        with canvas(device) as draw_obj:
            y_offset = 0

            for i, entry in enumerate(menu_entries[:-1]):  # Skip "Return"
                y = 5 + y_offset * 14

                if i == menu_index:
                    # Highlight selection
                    draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                    draw_obj.text((5, y), f"{entry.var_name}", font=font_small, fill="black")

                    # Show value if toggled
                    if showing_value:
                        y_offset += 1
                        y = 5 + y_offset * 14
                        if modifying_value:
                            draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                            draw_obj.text((5, y), f"{temp_value:.2f}", font=font_small, fill="black")
                        else:
                            draw_obj.text((5, y), f"{temp_value:.2f}", font=font_small, fill="white")

                else:
                    draw_obj.text((5, y), f"{entry.var_name}", font=font_small, fill="white")

                y_offset += 1

        # Check for button press
        if handle_button_submenu():
            if modifying_value:
                # Save new value and exit edit mode
                selected_entry.var_value = str(temp_value)
                modifying_value = False
                print(f"Saved new value: {temp_value}")
            elif showing_value:
                # Enter modify mode when pressing the value
                modifying_value = True
                print("Entering value edit mode")
            else:
                # Toggle showing the value
                showing_value = not showing_value
                print(f"Toggled value display: {showing_value}")

        # If in edit mode, update value with rotary input
        if modifying_value:
            temp_value = handle_rotation_edit_mode(temp_value)


def handle_rotation_edit_mode(temp_value):
    """
    Adjusts the currently selected value when in edit mode.
    """
    global last_clk, last_dt

    current_clk = clk.get_value()
    current_dt = dt.get_value()

    if last_clk == 1 and current_clk == 0:  # Detect falling edge
        if current_dt == 1:
            temp_value += 0.1  # Increase value
            print("Value Increased")
        else:
            temp_value -= 0.1  # Decrease value
            print("Value Decreased")

    last_clk = current_clk
    last_dt = current_dt

    return temp_value

def handle_button_submenu():
    """
    Handles button press detection.
    - Ensures proper debounce and press-release detection.
    """
    time.sleep(DEBOUNCE_TIME)  # Debounce

    if sw.get_value() == 0:  # Button is pressed
        print("SW Button Pressed")

        time.sleep(0.2)  # Extra debounce for stability

        # Wait for button release
        while sw.get_value() == 0:
            time.sleep(DEBOUNCE_TIME)

        return True  # Button was pressed

    return False  # No button press detected


# --- Main Loop ---

def main():
    global menu_index, enter_menu, last_index

    print("Rotary Encoder + Menu Example (Python + Luma.OLED + libgpiod)")

    try:
        while True:
            # Check for rotary encoder rotation
            handle_rotation()

            # Check if button is pressed
            handle_button()

            # Update display only if menu_index changed
            if menu_index != last_index:
                draw_menu()
                last_index = menu_index

            # If user pressed button:
            if enter_menu:
                enter_menu = False
                menu_selection_action()
                draw_menu()

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        chip.close()

# --- Rotary Encoder Handling ---

last_clk = clk.get_value()
last_dt = dt.get_value()

def handle_rotation():
    global menu_index, last_clk, last_dt

    current_clk = clk.get_value()
    current_dt = dt.get_value()

    if last_clk == 0 and current_clk == 1:  # Detect falling edge
        if current_dt == 1:
            menu_index -= 1  # Counter-clockwise
            print("Rotated Counter-Clockwise")
        else:
            menu_index += 1  # Clockwise
            print("Rotated Clockwise")

    last_clk = current_clk
    last_dt = current_dt

def handle_button():
    """
    Handles button press with proper debounce and press-release detection.
    """
    global enter_menu
    time.sleep(DEBOUNCE_TIME)  # Initial debounce

    if sw.get_value() == 0:  # Button is pressed (active-low)
        print("Button Pressed")
        time.sleep(0.2)  # Additional debounce for stability

        # Wait for button release
        while sw.get_value() == 0:
            time.sleep(DEBOUNCE_TIME)

        enter_menu = True  # Now it's safe to enter menu

# --- Run the Application ---

if __name__ == "__main__":
    main()
'''
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
VISIBLE_ITEMS = 4
DEBOUNCE_TIME = 0.01

# --- Initialize I2C OLED Display ---
try:
    serial = i2c(port=I2C_BUS, address=I2C_ADDRESS)
    device = sh1106(serial, rotate=0)
except Exception as e:
    print(f"Failed to initialize OLED display: {e}")
    sys.exit(1)

class MenuEntry:
    def __init__(self, var_name, var_value):
        self.var_name = var_name
        self.var_value = var_value
        self.showing_value = True  # Controls visibility
        self.editing_value = False  # Controls edit mode

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

# --- Menu Data ---
menu_entries = parse_config()
menu_index = 0
last_index = -1

# --- Fonts ---
font_small = ImageFont.load_default()
try:
    font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
except IOError:
    font_large = font_small

def draw_menu():
    global menu_index
    total_items = len(menu_entries)

    # Ensure menu_index is within valid bounds
    if menu_index < 0:
        menu_index = total_items - 1
    if menu_index >= total_items:
        menu_index = 0

    # Calculate the visible range dynamically
    start_index = max(0, menu_index - (menu_index % VISIBLE_ITEMS))
    end_index = min(start_index + VISIBLE_ITEMS, total_items)

    print(f"[DEBUG] Drawing menu - Index: {menu_index}, Showing Value: {menu_entries[menu_index].showing_value}, Editing: {menu_entries[menu_index].editing_value}, Visible Range: {start_index}-{end_index}")

    with canvas(device) as draw_obj:
        y_offset = 0
        for i in range(start_index, end_index):
            entry = menu_entries[i]
            y = 5 + y_offset * 14

            if i == menu_index:
                draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                draw_obj.text((5, y), f"{entry.var_name}", font=font_small, fill="black")

                # Show value below name
                if entry.showing_value:
                    y_offset += 1
                    y = 5 + y_offset * 14
                    value_text = f"{entry.var_value:.2f}"
                    if entry.editing_value:
                        draw_obj.rectangle((0, y - 2, device.width, y + 12), fill="white")
                        draw_obj.text((5, y), value_text, font=font_small, fill="black")
                    else:
                        draw_obj.text((5, y), value_text, font=font_small, fill="white")
            
            else:
                draw_obj.text((5, y), f"{entry.var_name}", font=font_small, fill="white")

            y_offset += 1

# --- Handle Rotation ---
last_clk = clk.get_value()
last_dt = dt.get_value()

def handle_rotation():
    global menu_index, last_clk, last_dt

    current_clk = clk.get_value()
    current_dt = dt.get_value()

    if last_clk == 1 and current_clk == 0:  # Detect falling edge (better for some encoders)
        if current_dt == 0:
            menu_index -= 1  # Clockwise
            print("[DEBUG] Rotated Clockwise (Index +1)")
        else:
            menu_index += 1  # Counter-clockwise
            print("[DEBUG] Rotated Counter-Clockwise (Index -1)")

    last_clk = current_clk
    last_dt = current_dt


# --- Handle Button Press ---
def handle_button():
    global menu_index

    if sw.get_value() == 0:
        time.sleep(0.2)
        while sw.get_value() == 0:
            time.sleep(DEBOUNCE_TIME)

        selected_entry = menu_entries[menu_index]

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

# --- Adjust Value While Editing ---
def handle_value_edit():
    global menu_index, last_clk, las

    current_clk = clk.get_value()
    current_dt = dt.get_value()

    selected_entry = menu_entries[menu_index]

    if last_clk == 1 and current_clk == 0:  # Detect falling edge
        if current_dt == 1:
            selected_entry.var_value += 0.1  # Increase value
            print("[DEBUG] Value Increased")
        else:
            selected_entry.var_value -= 0.1  # Decrease value
            print("[DEBUG] Value Decreased")

        draw_menu()  # Update screen when value changes

    last_clk = current_clk
    last_dt = current_dt

# --- Main Loop ---
def main():
    global menu_index, last_index

    print("Rotary Encoder + Menu Example (Python + Luma.OLED + libgpiod)")

    try:
        while True:
            handle_button()
            
            selected_entry = menu_entries[menu_index]
            if selected_entry.editing_value:
                handle_value_edit()
            else:
                handle_rotation()

            if menu_index != last_index:
                draw_menu()
                last_index = menu_index

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        chip.close()

if __name__ == "__main__":
    main()
