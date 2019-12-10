//-----------------------------------------------------------
//
// File: Vox_machina.ino
//
// Description: Scans buttons and touchpad, and sends MIDI messages on button/touch events.
//              Runs on Arduino Pro Micro (AT32u4) 5V 16MHz.
//              
//              Buttons:
//              --------
//              Vox Machina hardware includes 5 buttons numbered from 0 to 4, where button[0] is 
//              the small black button at the top, buttons 1 and 2 are the large green ones in the 
//              middle, and buttone 3 and 4 are the green ones at the bottom. The buttons are debounced and shadowed, so that we can trigger messages 
//              on press and/or release of each one independantly. We map these onto MIDI CC
//              Control Ids so that:
// 
//              Button 0 (black): Sets its control value to 127 while pressed and to 0 when unpressed.
// 
//              Buttons 1 & 2 (Green): These 2 together control the value of one Control Id.
//                                     when button 1 is pressed and released, the value of the
//                                     control is increases by one. Pressing and releasing button 2
//                                     lowers the value by one. 
// 
//              one Control Id, and the bottom two (blue) buttons similarly control the value
//              of a second Control Id. For each Control Id, pressing the upper button   
// 
//              Soft Pot:
//              ---------
//              Vox also includes a touch sensitive slider (soft pot) that is divided into 2 
//              virtual sliders, each generating CC value messages under its own CC Control Id. 
//              The value of each Control holds its value when the soft pot is untouched,
//              and tracks the touch position while the pot is touched. While the value is moving,
//              Vox sends a constant stream of CC value messages (at a preset interval). This means
//              messages are only sent when the pot is touched and the value is changing.
//   
//              Touching the top half of the soft pot causes Vox to send CC for the first device
//              whenever the touch value changes. When the touch is lifted, the last value 
//              sent considered to   and touching the bottom half controls the other. The soft port itself
//              only reports a single analog value from 0 to 1023, but this program sets boundaries
//              to define the upper and lower limits of each virtual slider.
//
//              Button LEDs:
//              ------------
//              All buttons except button[0] have a build in LED. For simplicity, we treat button 0 
//              as if it had a LED like the rest (it even has an assigned PWM output), but we don't 
//              use LED[0] for anything. The large button LEDs operate as a whole in one of 4 Flash 
//              Modes: 
//                      
//                       OFF   - Never on
//                       Short - Each time the button is pressed, the LED emists s quick flash.
//                               then stays out until the next press.
//                       Long  - Each press causes a longer flash.
//                       Fade  - Eqch press lights the LED, which then fades out
//                       On    - Never off
//               
//              In each of the modes above, the lit state of the LED is controlled by the brightness
//              value that can be set via config mode. 
//              
//              Config Mode:
//              ------------
//              There are 2 fundamental modes: Config mode and Normal Mode. Vox comes up in Normal 
//              Mode. To go from Normal Mode to Config Mode, hold down the bottom 2 buttons 
//              (button[3] and button[4]) until the middle 2 buttons light up (about 4 seconds). To
//              go from Config Mode to Normal Mode, press the top button (button[0]). 
// 
//              In Config mode, you can set the values of led_brightness and led_flashmode using 
//              the buttons. 
//              When you leave
//              Config Mode, the current values of led_brightness and led_flashmode are saved in 
//              EEPROM.
//
#include <frequencyToNote.h>
#include <MIDIUSB.h>
#include <MIDIUSB_Defs.h>
#include <pitchToFrequency.h>
#include <pitchToNote.h>
#include <EEPROM.h>

#define DEBUG 0
#define VERSION "Vox Machina v 1.0 (2019-10-06)"


// I/O Pins
#define SOFTPOT_PIN A1
  
const int btn_pin[5] = {2, 4, 7, 16, 8};  
const int led_pin[5] = {3, 5, 6, 10, 9};  


//--------[ MIDI Message Table ]--------

// Control Change Messages:
// 
//  | Header| Status| Data 1 | Data 2 |
//  +-------+-------+--------+--------+
//  | 0x0b  | 0xb0  | CtrlId | Value  |
// 
// Vox's Control ids are:
//    Upper slider(MSB): 0x01   Mod Wheel (MSB)
//    Lower slider(MSB): 0x10   GP Control 1 (MSB)
//    Button 0 Presss  : 0x11   Set GP control 02 to 1  
//    Button 0 Release : 0x11   Set GP control 02 to 0
//    Button 1 Press   : 0x12   GP Control 03 ++
//    Button 2 Press   : 0x12   GP Control 03 --
//    Button 3 Press   : 0x13   GP Control 04 ++
//    Button 4 Press   : 0x13   GP Control 04 --
//

const  midiEventPacket_t CC_MSG_BASE = {0x0b, 0xb0, 0x00, 0x00}; // template only - used to init message buffer

const uint8_t slider_control_id[] = {0x01, 0x10};
const uint8_t button_ctrl_id[]    = {0x11, 0x12, 0x12, 0x13, 0x13 }; 

uint8_t control_3_value; 
uint8_t control_4_value; 
      

//--------[ Saved config and EEPROM Addresses ]--------
unsigned int led_brightness;
enum fm {OFF, SLOW, FAST, FADE, ON} led_flashmode;

const unsigned int EEPROM_ADDR_FLASHMODE       = 0;
const unsigned int EEPROM_ADDR_BRIGHTNESS      = EEPROM_ADDR_FLASHMODE  + sizeof(led_flashmode);
const unsigned int EEPROM_ADDR_NEXT_AVAILABLE  = EEPROM_ADDR_BRIGHTNESS + sizeof(led_brightness);


//-------[ Durations and Intervals ]--------
const unsigned int BTN_TIMER_RESTART   =   10;  // Button debounce
const unsigned int BTN_REPEAT_DELAY    = 1000;  // Hold to repeat
const unsigned int BTN_REPEAT_INTERVAL =   50;  // Repeat rate while holding
const unsigned int CONFIG_TIMEOUT      = 4000;  // Hold to enter config mode

const unsigned int FLASH_DURATION_SLOW = 200;
const unsigned int FLASH_DURATION_FAST = 100;
const unsigned int FLASH_DURATION_FADE = 500;
const unsigned int FLASH_DURATION_ON   = 10000;

const unsigned int CONTROL_UPDATE_INTERVAL = 100;  // msec

//-----[ Touch Strip ]--------
const unsigned int POT_BOTTOM_0 = 0x010;
const unsigned int POT_TOP_0    = 0x1f8;
const unsigned int POT_BOTTOM_1 = 0x208;
const unsigned int POT_TOP_1    = 0x3f0;

const unsigned int SLIDER_FILTER_TIME = 5;

unsigned int pot_value;
unsigned int pot_filter;
unsigned int slider_timer[2];
uint8_t      slider_value[2];
uint8_t      prev_slider_value[2];
unsigned int control_change_timer;

//--------[ Buttons ]--------
unsigned int btn_status[5];
unsigned int btn_state[5];
unsigned int btn_prev_state[5];
unsigned int btn_timer[5];
unsigned int btn_rep_timer[5];
unsigned int led_timer[5];
unsigned int config_timer;

//--------[ System ]--------
unsigned int config_mode;
midiEventPacket_t cc_msg_buf;

//--------[ Timer ISR ]--------
unsigned long int last_ms;



void do_config_mode();
void do_normal_mode();
void send_msg();
void flash_led(int led);



//----------------------------------------------------------------------------
SIGNAL(TIMER0_COMPA_vect) {                             // 1 ms interrupt ISR
//----------------------------------------------------------------------------
   // Timer0 is already used for millis() - we'll just interrupt somewhere
   // in the middle.
   int            b;
   unsigned long current_ms;

  current_ms = millis();
  if (current_ms == last_ms) return;
   
  last_ms = current_ms;

  //  For all buttons...
  for (int i=0; i<5; i++) {

    // Button Debounce Timer
    b = !digitalRead(btn_pin[i]);
    if (b == btn_status[i]) {
      if ( btn_timer[i] ) {
        btn_timer[i]--;
      } else {
        btn_state[i] = btn_status[i];
      }
    } else {
      btn_timer[i]     = BTN_TIMER_RESTART;
    }
    btn_status[i] = b; 

    // Repeat timer
    if ( !btn_state[i]) {
      btn_rep_timer[i] = BTN_REPEAT_DELAY;
    }
    else if (btn_rep_timer[i]) {
      btn_rep_timer[i]--;
    }

    // LED Flash Timer
    if (!config_mode || i==3 || i==4) {
      // When in config mode, only do leds 3 & 4, since 1 & 2 are displaying brightness setting
      if (led_timer[i]) {
        // Timer is running. Fade in flashmode FADE
        if (led_flashmode == FADE) {
          b = led_brightness * led_timer[i]/FLASH_DURATION_FADE;
          analogWrite(led_pin[i], b);
        }
        // tick timer
        led_timer[i]--;
      }
      else if (led_flashmode == ON) {
        // Timer doesn't really do anything in flashmode ON
        led_timer[i] = FLASH_DURATION_ON;
      }
      else {
        // Not flashmode ON, and timer has expired, so turn led off
        analogWrite(led_pin[i], 0);
      }
    }
  }

  // Config Mode Timer
  if (btn_state[3] && btn_state[4]) {
    if (config_timer) {
      config_timer--; 
    }
  }
  else {
    config_timer = CONFIG_TIMEOUT;  
  }

  // Control Changes Timer
  if (control_change_timer) {
    control_change_timer--;
  }
}  // Timer ISR



//--------------------------------------------------------------
void setup() {
//--------------------------------------------------------------
  // Button and LED ports and button software status 
  for (int i = 0; i < 5; i++) {
      pinMode(led_pin[i], OUTPUT); 
      digitalWrite(led_pin[i], LOW);  
      pinMode(btn_pin[i], INPUT_PULLUP);  
      digitalWrite(btn_pin[i], HIGH);

      btn_status[i]    = 0;
      btn_timer[i]     = BTN_TIMER_RESTART;
      btn_state[i]     = 0;
      btn_rep_timer[i] = BTN_REPEAT_DELAY;
  }

  // Soft Pot
  pinMode(SOFTPOT_PIN, INPUT_PULLUP); 
  digitalWrite(SOFTPOT_PIN, HIGH);

  // Debug port
  if (true) {
    Serial.begin(57600); 
    while (!Serial) {
    }
  }

  //--------[ Load Config ]--------
  EEPROM.get(EEPROM_ADDR_BRIGHTNESS, led_brightness);
  EEPROM.get(EEPROM_ADDR_FLASHMODE,  led_flashmode);

  if (led_brightness < 0 || led_brightness > 250) {
    led_brightness = 128;
  }
  if (led_flashmode < 0 || led_flashmode > fm::ON) {
    led_flashmode = OFF;
  }

  config_mode    = 0;
  control_3_value = 0;
  control_4_value = 0;

  cc_msg_buf.header = CC_MSG_BASE.header;
  cc_msg_buf.byte1  = CC_MSG_BASE.byte1;
  cc_msg_buf.byte2  = CC_MSG_BASE.byte2;
  cc_msg_buf.byte3  = CC_MSG_BASE.byte3;

  //-------[ Startup Message ]--------
  for (int i=0; i<5; i++) {
    digitalWrite(led_pin[i], HIGH); 
    delay(100);
  }

  //--------[ 1ms Interrupt ]--------
  last_ms              = 0;
  config_timer         = CONFIG_TIMEOUT;
  control_change_timer = CONTROL_UPDATE_INTERVAL;

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  for (int i=0; i<5; i++) {
    flash_led(i);
  }

  if (true) {
    Serial.println(VERSION); 
    Serial.print("Flashmode = ");
    Serial.print(led_flashmode);
    Serial.print(", Brightness = ");
    Serial.println(led_brightness);
  }
}




//--------------------------------------------------------------
void loop(){
//--------------------------------------------------------------
  if (config_mode) {
    do_config_mode();

    if (DEBUG) {
      Serial.print("Flashmode: "); 
      Serial.print(led_flashmode);
      Serial.print(", Brightness: ");
      Serial.print(led_brightness);
      Serial.print(", prev[3]: ");
      Serial.print(btn_prev_state[3]);
      Serial.print(", state[3]: ");
      Serial.println(btn_state[3]);
      delay(200);
    }
  }
  else {
    do_normal_mode();
    if( false ) {
      Serial.print("Pot_value = ");
      Serial.print(pot_value, HEX);
      Serial.print(", pot_filter = ");
      Serial.print(pot_filter, HEX);
      Serial.print(", Slider 0 = ");
      Serial.print(slider_value[0], HEX);
      Serial.print(", Slider 1 = ");
      Serial.println(slider_value[1], HEX);
      delay(200);
    }
  }
}




//---------------------------------------------------------------------------------------------
//        Config Mode
//---------------------------------------------------------------------------------------------
void do_config_mode() {
  // Buttons 1 & 2 display the current brightness. buttons 3 & 4 display the current flashmode.

  analogWrite(led_pin[1], led_brightness);
  analogWrite(led_pin[2], led_brightness);
  analogWrite(led_pin[3], 0);
  analogWrite(led_pin[4], 0);

  // Button 0: Cancel config mode
  if (btn_state[0]) {
    analogWrite(led_pin[1], 0);
    analogWrite(led_pin[2], 0);
    EEPROM.put(EEPROM_ADDR_FLASHMODE,  led_flashmode);
    EEPROM.put(EEPROM_ADDR_BRIGHTNESS, led_brightness);
    config_mode = 0;
    config_timer = CONFIG_TIMEOUT;
  }
  btn_prev_state[0] = btn_state[0];

  //--------[ Button 1: Brighness Up ]--------
  if (btn_state[1] && (!btn_prev_state[1] || !btn_rep_timer[1])) {
    led_brightness = (led_brightness >= 250) ? 250 : led_brightness + 5;
    btn_rep_timer[1] = BTN_REPEAT_INTERVAL;
  } 
  btn_prev_state[1] = btn_state[1];

  //--------[ Button 2: Brighness Down ]--------
  if (btn_state[2] && (!btn_prev_state[2]) || !btn_rep_timer[2]) {
    led_brightness = (led_brightness <= 5) ? 5 : led_brightness - 5;
    btn_rep_timer[2] = BTN_REPEAT_INTERVAL;
  }
  btn_prev_state[2] = btn_state[2];

  //--------[ Button 3: Flashmode Up ]-------
  if (btn_state[3] && !btn_prev_state[3]) {
    switch (led_flashmode) {
      case  OFF:  led_flashmode = ON;    break;
      case  SLOW: led_flashmode = OFF;   break;
      case  FAST: led_flashmode = SLOW;  break;
      case  FADE: led_flashmode = FAST;  break;
      case  ON:   led_flashmode = FADE;  break;
      default:    led_flashmode = OFF;
    }
    flash_led(3);
    flash_led(4);
  }
  btn_prev_state[3] = btn_state[3];

  //--------[ Button 4: Flashmode Down ]-------
  if (btn_state[4] && !btn_prev_state[4]) {
    switch (led_flashmode) {
      case  OFF:  led_flashmode = SLOW; break;
      case  SLOW: led_flashmode = FAST; break;
      case  FAST: led_flashmode = FADE; break;
      case  FADE: led_flashmode = ON;   break;
      case  ON:   led_flashmode = OFF;  break;
      default:    led_flashmode = OFF;
    }

    for (int i=0; i<5; i++) {
      flash_led(i); 
    }
  }
  btn_prev_state[4] = btn_state[4];
}  // do_config_mode




//----------------------------------------------------------------------------------
//       Normal mode 
//----------------------------------------------------------------------------------
// Control Change Messages:
// 
//  | Header| Status| Data 1 | Data 2 |
//  +-------+-------+--------+--------+
//  | 0x0b  | 0xb0  | CtrlId | Value  |
// 
// Vox's Control ids are:
//    Upper slider(MSB): 0x01   Mod Wheel (MSB)
//    Lower slider(MSB): 0x10   GP Control 1 (MSB)
//    Button 0 Presss  : 0x11   Set GP control 02 to 1  
//    Button 0 Release : 0x11   Set GP control 02 to 0
//    Button 1 Press   : 0x12   GP Control 03 ++
//    Button 2 Press   : 0x12   GP Control 03 --
//    Button 3 Press   : 0x13   GP Control 04 ++
//    Button 4 Press   : 0x13   GP Control 04 --
//
void do_normal_mode() {
  // Check for request for config mode.
  if (!config_timer) {
    // Request for config mode
    config_mode = 1;
    btn_prev_state[3] = btn_state[3];
    btn_prev_state[4] = btn_state[4];
  }
  else {
    // No config request, so scan button states, looking for state changes.
    
    // Start with Button 0. Button 0 is special: it sends messages on both 
    // press and release, and there is no repeat
    cc_msg_buf.byte2 = button_ctrl_id[0];

    // Check for Button 0 press
    if (!btn_prev_state[0] && btn_state[0]) {
      cc_msg_buf.byte3 = 0x7f;
      send_msg(); 
    }

    // Check for Button 0 release
    if (btn_prev_state[0] && !btn_state[0]) {
      cc_msg_buf.byte3 = 0x00;
      send_msg(); 
   }
   btn_prev_state[0] = btn_state[0];

   // Now scan  buttons 1 to 4, with auto repeat on each

   for (int i = 1; i < 5; i++) {

    // Check for button press or button down and repeat timeout
    if (btn_state[i] && (!btn_prev_state[i] || !btn_rep_timer[i])) {
      cc_msg_buf.byte2  = button_ctrl_id[i];     

      if (!btn_rep_timer[i]) {
        btn_rep_timer[i] = BTN_REPEAT_INTERVAL;
      }

      flash_led(i);

      switch (i) {
        
        case 1: // Button 1 increments GP Control 3

          control_3_value =  (control_3_value >= 127) ? 127 : control_3_value + 1; 
          cc_msg_buf.byte3 = control_3_value; 
          send_msg(); 
          break;

        case 2: // Button 2 decrements GP Control 3
          control_3_value =  (control_3_value == 0) ? 0 : control_3_value - 1; 
          cc_msg_buf.byte3 = control_3_value; 
          send_msg(); 
          break;

        case 3:
          control_4_value =  (control_4_value >= 127) ? 127 : control_4_value + 1; 
          cc_msg_buf.byte3 = control_4_value; 
          send_msg(); 
          break;

        case 4:
          control_4_value =  (control_4_value == 0) ? 0 : control_4_value - 1; 
          cc_msg_buf.byte3 = control_4_value; 
          send_msg(); 
          break;
        }
      }
      btn_prev_state[i] = btn_state[i];
    }  // buttons 1-4


    // Check Membrane Pot. Typical sense values are
    //        No-touch:  0x3fa 
    //        Mid value: 0x1e9 to 0x230
    //        Min Value: 0x0e

    // Sample Touchpad, and filter it. 

    pot_value = analogRead(SOFTPOT_PIN);
    pot_filter = pot_value; // (pot_value + pot_filter)/2; 


    // In order to affect a slider, the filtered pot value has to be inside 
    // the Slider range until the the slider timer expires. Any sample outside 
    // the range restarts the timer.
    
    //--------[ Check range for Slider 0 ]-------- 
    if ((pot_filter < POT_BOTTOM_0)||( pot_filter > POT_TOP_0)) {
      // Not in slider 0 range
      slider_timer[0] = SLIDER_FILTER_TIME;
    } 
    else if( slider_timer[0]) {
      slider_timer[0]--;
    } 
    else {
      // Slider_timer[0] expired: Activate Slider[0]. 
      slider_value[0] = map(pot_filter, POT_BOTTOM_0, POT_TOP_0, 0, 127);
    }


    //--------[ Check against Slider 1 range ]--------
    if ((pot_filter < POT_BOTTOM_1) || (pot_filter > POT_TOP_1)) {
      // Not in Slider 1 range
      slider_timer[1] = SLIDER_FILTER_TIME;
    } 
    else if (slider_timer[1]) {
      slider_timer[1]--;
    } 
    else {
      // Slider_timer[1] expired: Activate Slider[1]
      slider_value[1] = map(pot_filter, POT_BOTTOM_1, POT_TOP_1, 0, 127);
    }

    // If the Control Change Timer has expired, look for control changes
    // and send updates.
    if (!control_change_timer) {

      for (int i=0; i<2; i++) {

        if (slider_value[i] != prev_slider_value[i]) {
          // Build & send a Control Change (CC) msg
          cc_msg_buf.byte2 = slider_control_id[i];
          cc_msg_buf.byte3 = slider_value[i]; 
          send_msg();
          prev_slider_value[i] = slider_value[i]; 
        }
      }
      control_change_timer = CONTROL_UPDATE_INTERVAL;
    }  //ctrl change timer
  }  // not config timer            
}


void send_msg() {
  MidiUSB.sendMIDI(cc_msg_buf);
  MidiUSB.flush();
 
  if (DEBUG) {
    Serial.print(cc_msg_buf.header, HEX);
    Serial.print(", ");
    Serial.print(cc_msg_buf.byte1, HEX);
    Serial.print(", ");
    Serial.print(cc_msg_buf.byte2, HEX);
    Serial.print(", ");
    Serial.println(cc_msg_buf.byte3, HEX);
  }
}


//------------------------
void flash_led(int led) {
//------------------------
  switch (led_flashmode) {
  case OFF: 
    analogWrite(led_pin[led],0);
    break;

  case SLOW:
    analogWrite(led_pin[led], led_brightness);
    led_timer[led] = FLASH_DURATION_SLOW;
    break;

  case FAST:
    analogWrite(led_pin[led], led_brightness);
    led_timer[led] = FLASH_DURATION_FAST;
    break;

  case FADE:
    analogWrite(led_pin[led], led_brightness);
    led_timer[led] = FLASH_DURATION_FADE;
    break;

  case ON:
    analogWrite(led_pin[led], led_brightness);
    led_timer[led] = FLASH_DURATION_ON;
  }

}



