// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <Wire.h>
// // The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// // Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
// //U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// // End of constructor list



// void setup(void) {
  
//   u8g2.begin(/* menu_select_pin= */ 5, /* menu_next_pin= */ 4, /* menu_prev_pin= */ 2, /* menu_home_pin= */ 3);

//   u8g2.setFont(u8g2_font_t0_11_tr);
// }

// struct menu_entry_type
// {
//   const uint8_t *font;
//   uint16_t icon;
//   const char *name;
// };

// struct menu_state
// {
//   int16_t menu_start;		/* in pixel */
//   int16_t frame_position;		/* in pixel */
//   uint8_t position;			/* position, array index */
// };

// /*
//   Icon configuration
//   Width and height must match the icon font size
//   GAP: Space between the icons
//   BGAP: Gap between the display border and the cursor.
// */
// #define ICON_WIDTH 32
// #define ICON_HEIGHT 32
// #define ICON_GAP 4
// #define ICON_BGAP 16
// #define ICON_Y 32+ ICON_GAP

// struct menu_entry_type menu_entry_list[] =
// {
//   { u8g2_font_open_iconic_embedded_4x_t, 65, "Clock Setup"},
//   { u8g2_font_open_iconic_embedded_4x_t, 66, "Gear Game"},
//   { u8g2_font_open_iconic_embedded_4x_t, 67, "Flash Light"},
//   { u8g2_font_open_iconic_embedded_4x_t, 68, "Home"},
//   { u8g2_font_open_iconic_embedded_4x_t, 72, "Configuration"},
//   { NULL, 0, NULL } 
// };

// int8_t button_event = 0;		// set this to 0, once the event has been processed

// void check_button_event(void)
// {
//   if ( button_event == 0 )
//     button_event = u8g2.getMenuEvent();
// }


// void draw(struct menu_state *state)
// {
//   int16_t x;
//   uint8_t i;
//   x = state->menu_start;
//   i = 0;
//   while( menu_entry_list[i].icon > 0 )  
//   {
//     if ( x >= -ICON_WIDTH && x < u8g2.getDisplayWidth() )
//     {
//       u8g2.setFont(menu_entry_list[i].font);
//       u8g2.drawGlyph(x, ICON_Y, menu_entry_list[i].icon );
//     }
//     i++;
//     x += ICON_WIDTH + ICON_GAP;
//     check_button_event();
//   }
//   u8g2.drawFrame(state->frame_position-1, ICON_Y-ICON_HEIGHT-1, ICON_WIDTH+2, ICON_WIDTH+2);
//   u8g2.drawFrame(state->frame_position-2, ICON_Y-ICON_HEIGHT-2, ICON_WIDTH+4, ICON_WIDTH+4);
//   u8g2.drawFrame(state->frame_position-3, ICON_Y-ICON_HEIGHT-3, ICON_WIDTH+6, ICON_WIDTH+6);
//   check_button_event();
// }


// void to_right(struct menu_state *state)
// {
//   if ( menu_entry_list[state->position+1].font != NULL )
//   {
//     if ( (int16_t)state->frame_position+ 2*(int16_t)ICON_WIDTH + (int16_t)ICON_BGAP < (int16_t)u8g2.getDisplayWidth() )
//     {
//       state->position++;
//       state->frame_position += ICON_WIDTH + (int16_t)ICON_GAP;
//     }
//     else
//     {
//       state->position++;      
//       state->frame_position = (int16_t)u8g2.getDisplayWidth() - (int16_t)ICON_WIDTH - (int16_t)ICON_BGAP;
//       state->menu_start = state->frame_position - state->position*((int16_t)ICON_WIDTH + (int16_t)ICON_GAP);
//     }
//   }
// }

// void to_left(struct menu_state *state)
// {
//   if ( state->position > 0 )
//   {
//     if ( (int16_t)state->frame_position >= (int16_t)ICON_BGAP+(int16_t)ICON_WIDTH+ (int16_t)ICON_GAP )
//     {
//       state->position--;
//       state->frame_position -= ICON_WIDTH + (int16_t)ICON_GAP;
//     }    
//     else
//     {
//       state->position--; 
//       state->frame_position = ICON_BGAP;
//       state->menu_start = state->frame_position - state->position*((int16_t)ICON_WIDTH + (int16_t)ICON_GAP);      
//     }
//   }
// }


// uint8_t towards_int16(int16_t *current, int16_t dest)
// {
//   if ( *current < dest )
//   {
//     (*current)++;
//     return 1;
//   }
//   else if ( *current > dest )
//   {
//     (*current)--;
//     return 1;
//   }
//   return 0;
// }

// uint8_t towards(struct menu_state *current, struct menu_state *destination)
// {
//   uint8_t r = 0;
//   uint8_t i;
//   for( i = 0; i < 6; i++ )
//   {
//     r |= towards_int16( &(current->frame_position), destination->frame_position);
//     r |= towards_int16( &(current->menu_start), destination->menu_start);
//   }
//   return r;
// }



// struct menu_state current_state = { ICON_BGAP, ICON_BGAP, 0 };
// struct menu_state destination_state = { ICON_BGAP, ICON_BGAP, 0 };

// void loop(void) {
//   do
//   {
//     u8g2.firstPage();
//     do
//     {
//       draw(&current_state);  
//       u8g2.setFont(u8g2_font_helvB10_tr);  
//       u8g2.setCursor((u8g2.getDisplayWidth()-u8g2.getStrWidth(menu_entry_list[destination_state.position].name))/2,u8g2.getDisplayHeight()-5);
//       u8g2.print(menu_entry_list[destination_state.position].name);
//       check_button_event();
//       delay(10);
//     } while( u8g2.nextPage() );
//     if ( button_event == U8X8_MSG_GPIO_MENU_NEXT )
//       to_right(&destination_state);
//     if ( button_event == U8X8_MSG_GPIO_MENU_PREV )
//       to_left(&destination_state);
//     if ( button_event == U8X8_MSG_GPIO_MENU_SELECT )
//     {
//       u8g2.setFont(u8g2_font_helvB10_tr);  
//       u8g2.userInterfaceMessage("Selection:", menu_entry_list[destination_state.position].name, "", " Ok ");
//     }
//     if ( button_event > 0 )	// all known events are processed, clear event
//       button_event = 0;
//   } while ( towards(&current_state, &destination_state) );
// }
#include <Arduino.h>
#include <U8g2lib.h>
#include "RotaryEncoder.h"

// LCD Menu Setup
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Rotary Encoder Pins
#define ROTARY_DT 2
#define ROTARY_CLK 3
#define ROTARY_SW 4

// Rotary Encoder Variables
int menuIndex = 0; // Current menu item index
int lastMenuIndex = -1;
bool enterMenu = false;

// Menu Entries
typedef struct menu_entry_type {
  const uint8_t *font;
  const char *name;
};

menu_entry_type menu_entry_list[] = {
  {u8g2_font_open_iconic_embedded_4x_t, "Clock Setup"},
  {u8g2_font_open_iconic_embedded_4x_t, "Gear Game"},
  {u8g2_font_open_iconic_embedded_4x_t, "Flash Light"},
  {u8g2_font_open_iconic_embedded_4x_t, "Home"},
  {u8g2_font_open_iconic_embedded_4x_t, "Configuration"},
  {u8g2_font_open_iconic_embedded_4x_t, "Speed"},
  {NULL, NULL} // End of menu
};

// Rotary Encoder
RotaryEncoder Rotary([]() {
  const unsigned int state = Rotary.GetState();
  if (state & DIR_CW) menuIndex--;
  if (state & DIR_CCW) menuIndex++;
}, ROTARY_DT, ROTARY_CLK, ROTARY_SW);

// void drawMenu() {
//   u8g2.setFont(u8g2_font_t0_11_tr); // Reset font to the smaller size for the menu
//   u8g2.firstPage();
//   do {
//     // Draw menu items
//     int totalItems = 0;
//     while (menu_entry_list[totalItems].name != NULL) totalItems++;

//     // Handle menu index bounds
//     if (menuIndex < 0) menuIndex = totalItems - 1;
//     if (menuIndex >= totalItems) menuIndex = 0;

//     for (int i = 0; i < totalItems; i++) {
//       int y = 16 + i * 12; // Adjust Y position for each menu item
//       if (i == menuIndex) {
//         // Highlight selected menu item
//         u8g2.drawBox(0, y - 10, u8g2.getDisplayWidth(), 12);
//         u8g2.setDrawColor(0); // Invert text color
//         u8g2.setCursor(5, y);
//         u8g2.print(menu_entry_list[i].name);
//         u8g2.setDrawColor(1); // Reset text color
//       } else {
//         u8g2.setCursor(5, y);
//         u8g2.print(menu_entry_list[i].name);
//       }
//     }
//   } while (u8g2.nextPage());
// }
void drawMenu() {
  const int visibleItems = 4; // Number of menu items to show at a time
  //u8g2.setFont(u8g2_font_6x12_tr); // Reset font to the smaller size for the menu
  u8g2.setFont(u8g2_font_t0_11_tr); // Reset font to the smaller size for the menu
  u8g2.firstPage();
  do {
    // Calculate total menu items
    int totalItems = 0;
    while (menu_entry_list[totalItems].name != NULL) totalItems++;

    // Handle menu index bounds
    if (menuIndex < 0) menuIndex = totalItems - 1;
    if (menuIndex >= totalItems) menuIndex = 0;

    // Calculate the start and end indices of the visible menu
    int startIndex = menuIndex - menuIndex % visibleItems; // Align to multiples of `visibleItems`
    if (startIndex + visibleItems > totalItems) {
      startIndex = totalItems - visibleItems; // Ensure we don't go out of bounds
      if (startIndex < 0) startIndex = 0;    // Handle case with fewer than `visibleItems`
    }
    int endIndex = startIndex + visibleItems;

    // Draw the visible menu items
    for (int i = startIndex; i < endIndex; i++) {
      int y = 16 + (i - startIndex) * 14; // Adjust Y position for each menu item
      if (i == menuIndex) {
        // Highlight the selected menu item
        u8g2.drawBox(0, y - 10, u8g2.getDisplayWidth(), 12);
        u8g2.setDrawColor(0); // Invert text color
        u8g2.setCursor(5, y);
        u8g2.print(menu_entry_list[i].name);
        u8g2.setDrawColor(1); // Reset text color
      } else {
        u8g2.setCursor(5, y);
        u8g2.print(menu_entry_list[i].name);
      }
    }

  } while (u8g2.nextPage());
}

void setup() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_t0_11_tr);
  Rotary.setup();
  Serial.begin(9600);
  Serial.println("Rotary Encoder + Menu Example");
}

void loop() {
  // Update rotary encoder state
  Rotary.loop();

  // Check for button press
  if (Rotary.GetButtonDown()) {
    enterMenu = true; // Set flag to enter menu
  }

  // Redraw menu only if index changes
  if (menuIndex != lastMenuIndex) {
    drawMenu();
    lastMenuIndex = menuIndex;
  }

  // Handle menu selection
  if (enterMenu) {
    enterMenu = false; // Clear flag
    Serial.print("Entering menu: ");
    Serial.println(menu_entry_list[menuIndex].name);

    // Placeholder for menu action
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB12_tr);
      u8g2.setCursor(5, 32);
      u8g2.print("Selected:");
      u8g2.setCursor(5, 48);
      u8g2.print(menu_entry_list[menuIndex].name);
    } while (u8g2.nextPage());
    delay(1000); // Pause for a moment to display selection
  }
}
