#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diag/trace.h"
#include "cmsis/cmsis_device.h"

/*--------------------------------------------------------------------------------

                ECE 355 - Microprocessor-Based Systems
                          Fall 2024

            PWM Signal Generation and Monitoring System

               -| Karanbir Gosal & Tanvir Kahlon |-

--------------------------------------------------------------------------------*/

#pragma GCC diagnostic push //Save Current State
#pragma GCC diagnostic ignored "-Wunused-parameter" //Ignore warnings for unused functions
#pragma GCC diagnostic ignored "-Wmissing-declarations" //Ignore warnings for non prior declarations
#pragma GCC diagnostic ignored "-Wreturn-type" // Ignore when no return statement used

//-------------------- Declaration Section -----------------------

//Initialize and control the SPI communication with the OLED display.
SPI_HandleTypeDef SPI_Handle;

// Sends a single byte to the OLED display
void oled_Write(unsigned char);

// Sends a command byte to the OLED to control the behavior of the display, such as
// contrast or display mode.
void oled_Write_Cmd(unsigned char cmd);

//Sends a data byte to the OLED for actual content
void oled_Write_Data(unsigned char data);

// Global variable to track if a timer-triggered event has occurred.
int timerTriggered = 0;

//Initialize and configure the OLED display.
void oled_config(void);

//Updates the OLED display with new information.
void refresh_OLED(void);

// Global variable to track the current edge state(1st or 2nd edge) for frequency
int rising_edge = 0;

// Global variable for the calculated frequency of input signal.
int Freq = 0;

// Global variable to store the calculated resistance value.
int Res = 0;


//Used to introduce a small delay or synchronize code.
static inline void __nop(void) {
    __asm("nop"); // Assembler instruction for no operation
}

//----------LED Display Initialization --------------------

unsigned char oled_init_cmds[] =
{
    0xAE,
    0x20, 0x00,
    0x40,
    0xA0 | 0x01,
    0xA8, 0x40 - 1,
    0xC0 | 0x08,
    0xD3, 0x00,
    0xDA, 0x32,
    0xD5, 0x80,
    0xD9, 0x22,
    0xDB, 0x30,
    0x81, 0xFF,
    0xA4,
    0xA6,
    0xAD, 0x30,
    0x8D, 0x10,
    0xAE | 0x01,
    0xC0,
    0xA0
};


//
// Character specifications for LED Display (1 row = 8 bytes = 1 ASCII character)
// Example: to display '4', retrieve 8 data bytes stored in Characters[52][X] row
//          (where X = 0, 1, ..., 7) and send them one by one to LED Display.
// Row number = character ASCII code (e.g., ASCII code of '4' is 0x34 = 52)
//
unsigned char Characters[][8] = {
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b01011111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // !
    {0b00000000, 0b00000111, 0b00000000, 0b00000111, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // "
    {0b00010100, 0b01111111, 0b00010100, 0b01111111, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // #
    {0b00100100, 0b00101010, 0b01111111, 0b00101010, 0b00010010,0b00000000, 0b00000000, 0b00000000},  // $
    {0b00100011, 0b00010011, 0b00001000, 0b01100100, 0b01100010,0b00000000, 0b00000000, 0b00000000},  // %
    {0b00110110, 0b01001001, 0b01010101, 0b00100010, 0b01010000,0b00000000, 0b00000000, 0b00000000},  // &
    {0b00000000, 0b00000101, 0b00000011, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // '
    {0b00000000, 0b00011100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // (
    {0b00000000, 0b01000001, 0b00100010, 0b00011100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // )
    {0b00010100, 0b00001000, 0b00111110, 0b00001000, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // *
    {0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // +
    {0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ,
    {0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // -
    {0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // .
    {0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // /
    {0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // 0
    {0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // 1
    {0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // 2
    {0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // 3
    {0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000,0b00000000, 0b00000000, 0b00000000},  // 4
    {0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001,0b00000000, 0b00000000, 0b00000000},  // 5
    {0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000,0b00000000, 0b00000000, 0b00000000},  // 6
    {0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // 7
    {0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // 8
    {0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110,0b00000000, 0b00000000, 0b00000000},  // 9
    {0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // :
    {0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ;
    {0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // <
    {0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // =
    {0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // >
    {0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // ?
    {0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // @
    {0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110,0b00000000, 0b00000000, 0b00000000},  // A
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // B
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010,0b00000000, 0b00000000, 0b00000000},  // C
    {0b01111111, 0b01000001, 0b01000001, 0b00100010, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // D
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // E
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // F
    {0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010,0b00000000, 0b00000000, 0b00000000},  // G
    {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // H
    {0b01000000, 0b01000001, 0b01111111, 0b01000001, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // I
    {0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // J
    {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // K
    {0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // L
    {0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // M
    {0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // N
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // O
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // P
    {0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110,0b00000000, 0b00000000, 0b00000000},  // Q
    {0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // R
    {0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // S
    {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // T
    {0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // U
    {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111,0b00000000, 0b00000000, 0b00000000},  // V
    {0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // W
    {0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011,0b00000000, 0b00000000, 0b00000000},  // X
    {0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // Y
    {0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011,0b00000000, 0b00000000, 0b00000000},  // Z
    {0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // [
    {0b00010101, 0b00010110, 0b01111100, 0b00010110, 0b00010101,0b00000000, 0b00000000, 0b00000000},  // back slash
    {0b00000000, 0b00000000, 0b00000000, 0b01000001, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // ]
    {0b00000100, 0b00000010, 0b00000001, 0b00000010, 0b00000100,0b00000000, 0b00000000, 0b00000000},  // ^
    {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // _
    {0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // `
    {0b00100000, 0b01010100, 0b01010100, 0b01010100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // a
    {0b01111111, 0b01001000, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // b
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // c
    {0b00111000, 0b01000100, 0b01000100, 0b01001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // d
    {0b00111000, 0b01010100, 0b01010100, 0b01010100, 0b00011000,0b00000000, 0b00000000, 0b00000000},  // e
    {0b00001000, 0b01111110, 0b00001001, 0b00000001, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // f
    {0b00001100, 0b01010010, 0b01010010, 0b01010010, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // g
    {0b01111111, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // h
    {0b00000000, 0b01000100, 0b01111101, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // i
    {0b00100000, 0b01000000, 0b01000100, 0b00111101, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // j
    {0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // k
    {0b00000000, 0b01000001, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // l
    {0b01111100, 0b00000100, 0b00011000, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // m
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // n
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // o
    {0b01111100, 0b00010100, 0b00010100, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // p
    {0b00001000, 0b00010100, 0b00010100, 0b00011000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // q
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // r
    {0b01001000, 0b01010100, 0b01010100, 0b01010100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // s
    {0b00000100, 0b00111111, 0b01000100, 0b01000000, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // t
    {0b00111100, 0b01000000, 0b01000000, 0b00100000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // u
    {0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // v
    {0b00111100, 0b01000000, 0b00111000, 0b01000000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // w
    {0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // x
    {0b00001100, 0b01010000, 0b01010000, 0b01010000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // y
    {0b01000100, 0b01100100, 0b01010100, 0b01001100, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // z
    {0b00000000, 0b00001000, 0b00110110, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // {
    {0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // |
    {0b00000000, 0b01000001, 0b00110110, 0b00001000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // }
    {0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // ~
    {0b00001000, 0b00011100, 0b00101010, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000}   // <-
};


// delay_ms function is used to create a delay
// equal to the specified number of milliseconds.

void delay_ms(uint32_t ms) {

    volatile uint32_t count;
    for (uint32_t i = 0; i < ms; i++) {
        for (count = 0; count < 16000; count++) {
            // No operation acts as a delay.
        }
    }
}

// Updates the OLED display with the latest measured values
// for resistance and frequency.

void refresh_OLED(void)
{
   // Declare a buffer to hold the text string that will be displayed.
   // The buffer size is 17 to accommodate up to 16 characters plus a null
   // terminator ('\0') for the end of the string.
   unsigned char Buffer[17];

   // Print the project title
   snprintf(Buffer, sizeof(Buffer), "ECE 355 PROJECT");

   // Page as Row
   // Set the display location for the resistance text on the OLED.
   // Command 0xB0 sets the OLED to page 0, where the resistance text will start.
   oled_Write_Cmd(0xB0); // Set page 0 (first row of text display)
   oled_Write_Cmd(0x02); // Set lower column address to 2 (for spacing)
   oled_Write_Cmd(0x10); // Set higher column address (depends on display configuration)

   // Send each character in the Buffer to the OLED display, one by one.
   for (uint8_t c = 0; c < 17; c++) {
	   // Stop if we reach the end of the string (null terminator '\0').
	   if (Buffer[c] == '\0') break;

	   // For each character, we send 8 bytes of data, each byte representing one
	   // row of pixels in an 8x8 grid for that character.
	   for (uint8_t byte = 0; byte < 8; byte++) {
		   oled_Write_Data(Characters[Buffer[c]][byte]); // Send character data to OLED
	   }
   }

   // Print an empty line
   snprintf(Buffer, sizeof(Buffer), "");

   oled_Write_Cmd(0xB1); // Set page 1
   oled_Write_Cmd(0x02);
   oled_Write_Cmd(0x10);

   for (uint8_t c = 0; c < 17; c++) {
    if (Buffer[c] == '\0') break;

    for (uint8_t byte = 0; byte < 8; byte++) {
	   oled_Write_Data(Characters[Buffer[c]][byte]);
    }
   }

   // Print the resistance value
   snprintf(Buffer, sizeof(Buffer), "R: %5u Ohms", Res);

   oled_Write_Cmd(0xB2); // Set page 2
   oled_Write_Cmd(0x02);
   oled_Write_Cmd(0x10);

   for (uint8_t c = 0; c < 17; c++) {
       if (Buffer[c] == '\0') break;

       for (uint8_t byte = 0; byte < 8; byte++) {
           oled_Write_Data(Characters[Buffer[c]][byte]);
       }
   }

   // Print the Frequency
   snprintf(Buffer, sizeof(Buffer), "F: %5u Hz", Freq);

   oled_Write_Cmd(0xB3); // Set page 3
   oled_Write_Cmd(0x02);
   oled_Write_Cmd(0x10);

   for (uint8_t c = 0; c < 17; c++) {
       if (Buffer[c] == '\0') break;

       for (uint8_t byte = 0; byte < 8; byte++) {
           oled_Write_Data(Characters[Buffer[c]][byte]);
       }
   }

   delay_ms(100);
}


// Sends a command byte to the OLED display

void oled_Write_Cmd(unsigned char cmd)
{
    // Set PB6 (CS#) to HIGH to indicate that no transmission is currently active.
    // CS# (Chip Select) going HIGH indicates the end of a previous transaction
    // or a pause before starting a new one.
    GPIOB->ODR |= GPIO_ODR_6;

    // Set PB7 (D/C#) to LOW to indicate a command is being sent (D/C# = 0 for commands).
    // D/C# (Data/Command) pin differentiates between command (0) and data (1) modes.
    GPIOB->ODR &= ~GPIO_ODR_7;

    // Set PB6 (CS#) to LOW to begin a new transmission.
    // Setting CS# to LOW activates the SPI communication for the OLED,
    // indicating that the display should receive data.
    GPIOB->ODR &= ~GPIO_ODR_6;

    // Call the oled_Write function to send the command byte to the OLED display.
    // oled_Write sends the byte over SPI, using the configurations set by this function.
    oled_Write(cmd);

    // Set PB6 (CS#) back to HIGH to end the current transmission.
    // This step signals the OLED that the transmission of the command is complete.
    GPIOB->ODR |= GPIO_ODR_7;
}


// Sends a data byte to the OLED display

void oled_Write_Data(unsigned char data)
{
    // Set PB6 (CS#) to HIGH to signal the end of any previous transmission.
    // CS# (Chip Select) going HIGH indicates that there is no active communication,
    // ensuring that any previous SPI transmission has completed.
    GPIOB->ODR |= GPIO_ODR_6;

    // Set PB7 (D/C#) to HIGH to indicate that we are sending data.
    // D/C# (Data/Command) pin differentiates between command (0) and data (1).
    // Setting it HIGH here signifies that the byte is display data, not a command.
    GPIOB->ODR |= GPIO_ODR_7;

    // Set PB6 (CS#) to LOW to initiate a new SPI transmission.
    // Setting CS# to LOW signals the OLED to start receiving the data.
    GPIOB->ODR &= ~GPIO_ODR_6;

    // Call the oled_Write function to send the data byte to the OLED display.
    // oled_Write sends the byte over SPI, according to the configurations set by this function.
    oled_Write(data);

    // Set PB6 (CS#) back to HIGH to end the current transmission.
    // This indicates that the transmission of the data byte is complete.
    GPIOB->ODR |= GPIO_ODR_7;
}



// sends a single byte of data or command to the OLED display via SPI.


void oled_Write(unsigned char Value)
{
    // Wait until the SPI1 peripheral is ready to transmit (TXE flag set in SPI1_SR).
    // The TXE (Transmit Buffer Empty) flag in SPI1's status register (SPI1_SR)
    // indicates that the transmit buffer is empty and ready for a new byte.
    // The function `__HAL_SPI_GET_FLAG` checks this flag.
    while (!__HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE))
    {
        // Busy-wait loop: do nothing until TXE becomes 1 (buffer is empty and ready).
    }


    HAL_SPI_Transmit(&SPI_Handle, &Value, 1, HAL_MAX_DELAY);

    // buffer is empty and that the transmission is complete.
    // This prevents further operations until the SPI
    // transmission has definitely finished.
    while (!__HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE))
    {
        // Busy-wait loop: do nothing until TXE becomes 1 (transmission is complete).
    }
}



// oled_config configures the necessary GPIO pins, enables SPI communication, and sends
// initialization commands to set up the OLED display.

void oled_config()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable clocks for GPIOB Port B (data/control lines) and SPI1 (send interface) peripherals.
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PB3 and PB5 for SPI communication.
    // PB3 and PB5 are configured as alternate function pins to be used for SPI1.
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            // Set to alternate function push-pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;    // Set speed to medium
    GPIO_InitStruct.Pull = GPIO_NOPULL;                // No pull-up or pull-down
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;         // Set alternate function for SPI1
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PB4, PB6, and PB7 as general output pins for control signals.
    // PB4, PB6, and PB7 are used to control signals to the OLED and  shift register
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;        // Set to output push-pull to drive OLED
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;    // Set speed to medium
    GPIO_InitStruct.Pull = GPIO_NOPULL;                // No pull-up or pull-down
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure SPI settings.
    // Assign SPI1 to the SPI handle and set up various SPI parameters.
    SPI_Handle.Instance = SPI1; // For comms
    SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;      // 1-line communication
    SPI_Handle.Init.Mode = SPI_MODE_MASTER;               // Set as SPI master
    SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;         // 8-bit data size
    SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;       // Clock polarity low when idle
    SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;           // Data sampled on first clock edge
    SPI_Handle.Init.NSS = SPI_NSS_SOFT;                   // Chip Select management
    SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // Set clock prescaler
    SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;          // Transmit MSB first
    SPI_Handle.Init.CRCPolynomial = 7;                    // CRC polynomial (unused here)

    // Initialize the SPI interface with the specified settings.
    HAL_SPI_Init(&SPI_Handle);

    // Enable the SPI peripheral.
    __HAL_SPI_ENABLE(&SPI_Handle); //Data starts

    // Perform a hardware reset on the OLED display using PB4 for a consistent state
    // Set PB4 LOW, wait, then set PB4 HIGH, waiting again after each change.
    GPIOB->BRR = GPIO_PIN_4;               // Set PB4 to 0 (reset the OLED)
    for (volatile int i = 0; i < 1000000; i++) { __nop(); } // Short delay
    GPIOB->BSRR = GPIO_PIN_4;              // Set PB4 to 1 (end reset)
    for (volatile int i = 0; i < 1000000; i++) { __nop(); } // Short delay

    // Send initialization commands to configure the OLED display.
    for (unsigned int i = 0; i < sizeof(oled_init_cmds); i++) {
        oled_Write_Cmd(oled_init_cmds[i]);
    }

    // Clear display by filling its data memory with zeros.
    // This loop writes 0s to all segments in each of the 8 pages of the display.
    for (uint8_t page = 0; page < 8; page++) {
        oled_Write_Cmd(0xB0 | page);    // Set page address (e.g., 0xB0 for page 0)
        oled_Write_Cmd(0x00);           // Set lower column address to 0
        oled_Write_Cmd(0x10);           // Set higher column address to 0

        // Write 128 zeros across the current page to clear the display.
        for (uint8_t seg = 0; seg < 128; seg++) {
            oled_Write_Data(0x00);      // Write zero to clear the segment
        }
    }
}


// ADC_Config configures and initializes the ADC (Analog-to-Digital Converter) to
// continuously sample analog input from the channel - PA5.

// This digital value is used to control the DAC and display the resistance on the OLED screen.

static void ADC_Config()
{
    // Enable the clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Enable the clock for the ADC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Configure the GPIO pin for analog input
    GPIOC->MODER &= 0xFFFFFFF3;     // Clear bits for PA5
    GPIOC->MODER |= 0x0000000C;     // Set bits for PA5 to analog mode (11)

    // Set the GPIO pin configuration to no pull-up or pull-down.
    GPIOC->PUPDR &= 0xFFFFFFF3;
    GPIOC->PUPDR |= 0x00000000;     // No pull-up/pull-down for analog mode

    // Configure the ADC for continuous conversion mode and overrun mode.

    // Continuous mode enables repeated conversions, and overrun mode automatically
    // overwrites previous data if it is not read in time.
    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;

    // Channel 5 is connected to the analog input pin.
    ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

    // Set the ADC sample time to the maximum (239.5 ADC clock cycles) for higher accuracy.
    ADC1->SMPR &= ~((uint32_t)0x00000007); // Clear sampling time bits
    ADC1->SMPR |= (uint32_t)0x00000007;    // Set sample time to 239.5 cycles

    // Enable the ADC by setting the ADEN bit
    ADC1->CR |= (uint32_t)ADC_CR_ADEN;

    // Wait for the ADC to complete its initialization.

    // The ADEN flag in the ISR (Interrupt and Status Register) will be set once
    // the ADC is ready for conversions.
    while (!(ADC1->ISR & ADC_CR_ADEN))
    {
        // Busy-wait loop: Do nothing until the ADC is fully enabled.
    }
}

// DAC (Digital-to-Analog Converter):
// Outputs an analog signal based on the digital potentiometer reading from the ADC.
// This DAC output is fed to the Optocoupler, which controls the NE555 timer's frequency and duty cycle.
//
// DAC_Config configures and initializes the DAC to generate
// an analog output signal based on digital input values. This function is
// called once during initialization to set up the DAC for operation.


static void DAC_Config()
{
    // Enable the clock
    RCC->AHBENR |= ((uint32_t)0x00020000);

    // Enable the clock, which is used for timing and triggering DAC
    RCC->APB1ENR |= ((uint32_t)0x20000000);

    // Configure GPIOA for analog mode to serve as the DAC output.
    // Clear the mode bits and set it to analog mode.
    GPIOA->MODER &= 0xFFFFFCFF;     // Clear bits
    GPIOA->MODER |= 0x00000300;     // Set to analog mode (11 in MODER bits)

    // Configure pins for no pull-up or pull-down resistors to prevent interfering with analog signals.
    GPIOA->PUPDR &= 0xFFFFFCFF;     // Clear PUPDR bits
    GPIOA->PUPDR |= 0x00000000;     // No pull-up/pull-down for analog mode

    // Configure the DAC control register to enable channel 1
    // Ensure that channel 1 of the DAC is ready for output.
    DAC->CR &= 0xFFFFFFFF9;         // Clear relevant bits (setup for channel 1)

    // Restart channel 1 by clearing and setting the relevant enable bits.
    DAC->CR |= 0x00000000;          // Clear channel settings
    DAC->CR |= 0x00000001;          // Enable DAC channel 1
}


// myTIM2_Init initializes TIM2 to count pulses and generate an interrupt on overflow.
// This function enabling TIM2 to measure pulse width or frequency by counting timer ticks.

void myTIM2_Init()
{
    /* Enable the clock for the TIM2 peripheral in the APB1 bus. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2 Control Register 1 (TIM2->CR1):
       - Enable buffer auto-reload (ARPE)
       - Count up mode: counter counts from 0 up to the value in ARR.
       - Stop on overflow: the counter stops counting on overflow (auto-reload).
       - Enable update events: allows update events on counter overflow.
       - Interrupt on overflow only: interrupt generated only on counter overflow. */
    TIM2->CR1 = ((uint16_t)0x008C);

    /* Set the prescaler value for TIM2.
       The prescaler divides the input clock frequency to control the timer’s counting rate. */
    TIM2->PSC = ((uint16_t)0x0000);

    /* Set the auto-reload register to its maximum value.
       This value defines the upper limit of the counter before it overflows and resets.*/
    TIM2->ARR = ((uint32_t)0xFFFFFFFF);

    /* Generate an update event to load the prescaler value into the timer.*/
    TIM2->EGR = ((uint16_t)0x0001);

    /* Set the TIM2 interrupt priority to the highest level (0).*/
    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable the TIM2 interrupt in the NVIC.
       This allows the TIM2 interrupt to be recognized and processed by the CPU.*/
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation in TIM2.
       This allows TIM2 to generate an interrupt when the counter overflows. */
    TIM2->DIER |= TIM_DIER_UIE;

    /* Start the TIM2 timer by enabling the counter.
       This allows TIM2 to start counting timer pulses. */
    // TIM2->CR1 |= TIM_CR1_CEN;
}


// myEXTI_Init configures external interrupts for specific pins on the micro-controller,
// enabling the system to detect and respond to events on those pins, such as
// rising edges from signal sources.

//In this configuration, it sets up external
// interrupts on PA0 and PA2 to handle events like button presses or signal changes.

void myEXTI_Init()
{
    /* Map EXTI line 1 to PA1.
       This sets PA1 as the source for EXTI1*/
    SYSCFG->EXTICR[0] = 0x00000000;

    /* Configure EXTI lines 0 and 2 to trigger on the rising edge.
       This will cause an interrupt when a rising edge is detected on PA0 (USER button)
       or PA2 (function generator signal)*/
    EXTI->RTSR |= ((uint32_t)0x00000005);

    /* Enable interrupts on EXTI lines 0 and 2.*/
    EXTI->IMR |= (EXTI_IMR_MR2 | EXTI_IMR_MR0);

    /* Set the priority of EXTI0_1 interrupt line to the highest (0),*/
    NVIC_SetPriority(EXTI0_1_IRQn, 0);

    /* Set the priority of EXTI2_3 interrupt line to a lower priority (1)*/
    NVIC_SetPriority(EXTI2_3_IRQn, 1);

    /* Enable interrupts in the NVIC */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
}


// myGPIOA_Init configures the pins PA2 and PA0 on GPIOA as inputs without any pull-up or pull-down resistors.

void myGPIOA_Init()
{
    /* Enable the clock*/
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Configure PA2 and PA0 as input pins by clearing their mode bits.
       Clearing these bits (setting them to 00) configures PA2 and PA0 as input.*/
    GPIOA->MODER &= ~(GPIO_MODER_MODER2);
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);  // Set PA0 as input (clear MODER bits for PA0)

    /* Ensure that no pull-up or pull-down resistors are enabled for PA2 and PA0.
       Used to avoid floating states
       - 00 = No pull-up, pull-down */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);  // No pull-up/pull-down for PA2
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);  // No pull-up/pull-down for PA0
}



// Interrupt handler for TIM2 update events, which are triggered when TIM2 overflows or reaches
// its auto-reload value. This function clears the update interrupt flag and restarts the timer.


void TIM2_IRQHandler()
{
    /* Check if the update interrupt flag (UIF) is set */
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        /* Clear the update interrupt flag by resetting UIF in TIM2 status register. */
        TIM2->SR &= ~(TIM_SR_UIF);

        /* Restart the timer by setting the counter enable (CEN) bit in TIM2 control register.*/
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

// For User Button
#define TRUE (1==1)
#define FALSE (1==0)
#define BUTTON_PUSHED TRUE
#define BUTTON_RELEASED FALSE
static int button_state = BUTTON_RELEASED;

// EXTI0_1_IRQHandler handles external interrupts on EXTI lines 0 and 1,
// which are connected to PA0 and PA1.

void EXTI0_1_IRQHandler()
{
    double period = 0;
    double frequency = 0;

    /* Check if EXTI0 interrupt pending flag is set.
       This flag indicates that a rising edge was detected on PA0 (connected to EXTI0).*/
    if ((EXTI->PR & EXTI_PR_PR0) != 0)
    {
    	if (button_state == BUTTON_RELEASED)  // Only execute if button hasn't been pressed yet
    	   {
			//Toggle the interrupt mask for EXTI1 and EXTI2.
			EXTI->IMR ^= ((EXTI_IMR_MR2) | (EXTI_IMR_MR1));

			//Toggle the rising edge trigger for EXTI1 and EXTI2.
			EXTI->RTSR ^= ((uint32_t)0x00000006);

			button_state = BUTTON_PUSHED;
			trace_printf("Button Pushed\n");
    	   }
    	else {
    		button_state = BUTTON_RELEASED;
    	}
        // Clear the pending interrupt flag for EXTI0.
        EXTI->PR = EXTI_PR_PR0;

    }



    /* Check if EXTI1 interrupt pending flag is set.
       This flag indicates that a rising edge was detected on PA1 (connected to EXTI1). */
    if ((EXTI->PR & EXTI_PR_PR1) != 0)
    {
        // Check if it is the first or second rising edge.
        if (rising_edge == 1)
        {
            // First rising edge: Calculate the period and frequency
            rising_edge = 0;  // Reset the edge tracker for the next cycle

            // Mask EXTI1 to prevent further interrupts until this calculation is complete
            EXTI->IMR &= ~(EXTI_IMR_MR1);

            // Stop TIM2 to capture the count value
            TIM2->CR1 &= ~(TIM_CR1_CEN);

            // Read the current count from TIM2, which represents the time elapsed since the last rising edge
            uint32_t count = TIM2->CNT;

            // Calculate the period based on the count value and the system clock
            period = (double)count / (double)SystemCoreClock;

            // Calculate the frequency from the period
            frequency = 1.0 / period;

            // Update the global variable `Freq` with the calculated frequency
            Freq = frequency;
            //trace_printf("Signal frequency: %.2f Hz\n", 1.0 * SystemCoreClock / count);

            // Re-enable EXTI1 interrupt
            EXTI->IMR |= EXTI_IMR_MR1;

            // Clear the pending interrupt flag for EXTI1
            EXTI->PR |= EXTI_PR_PR1;
        }
        else
        {
            // Second rising edge: Start counting the period

            // Set the edge tracker to indicate this is the first edge in a new cycle
            rising_edge = 1;

            // Reset the TIM2 counter to zero
            TIM2->CNT = 0x00000000;

            // Start TIM2 to begin counting time for the next period measurement
            TIM2->CR1 |= TIM_CR1_CEN;

            // Clear the pending interrupt flag for EXTI1
            EXTI->PR |= EXTI_PR_PR1;
        }
    }
}


// EXTI2_3_IRQHandler handles external interrupts on EXTI lines 2 and 3,
// which are connected to PA2 and PA3.


void EXTI2_3_IRQHandler()
{
    double period = 0;
    double frequency = 0;

    /* Check if the EXTI2 interrupt pending flag is set.
       This indicates that a rising edge was detected on PA2 (connected to EXTI2) */
    if ((EXTI->PR & EXTI_PR_PR2) != 0)
    {
        // Stop TIM2 temporarily to capture its current count value
        TIM2->CR1 &= ~(TIM_CR1_CEN);

        // Check if this is the first edge or the second edge
        if (timerTriggered == 0)
        {
            // First edge: Reset and start TIM2 for the next measurement
            TIM2->CNT = ((uint32_t)0x1);   // Reset the TIM2 counter to 1
            TIM2->CR1 |= TIM_CR1_CEN;      // Start TIM2
            timerTriggered = 1;            // Set flag to indicate timer has been started
        }
        else
        {
            // Second edge: Calculate the frequency based on the elapsed timer count
            TIM2->CR1 &= ~(TIM_CR1_CEN);   // Stop TIM2
            EXTI->IMR &= ~(EXTI_IMR_MR2);  // Mask EXTI2 to prevent further interrupts during calculation

            // Get the current count from TIM2, which represents the time since the first rising edge
            uint32_t count = TIM2->CNT;

            // Calculate the period based on the count and the system clock frequency
            period = (double)count / (double)SystemCoreClock;

            // Calculate the frequency as the inverse of the period
            frequency = 1.0 / period;

            // Update the global variable `Freq` with the calculated frequency
            Freq = frequency;

            // Clear and reset variables for the next measurement
            timerTriggered = 0;            // Reset the trigger flag for the next cycle
            EXTI->IMR |= (EXTI_IMR_MR2);   // Unmask EXTI2 to allow further interrupts
        }

        // Clear the EXTI2 interrupt pending flag to allow the next interrupt
        EXTI->PR = EXTI_PR_PR2;
    }
}

void SystemClock48MHz(void)
{
    // Disable the PLL to allow configuration
    RCC->CR &= ~(RCC_CR_PLLON);

    // Wait until the PLL is completely turned off and unlocked
    while ((RCC->CR & RCC_CR_PLLRDY) != 0);

    // Configure the PLL for a 48 MHz system clock
    RCC->CFGR = 0x00280000;

    // Enable the PLL after configuring it
    RCC->CR |= RCC_CR_PLLON;

    // Wait until the PLL is locked and ready for use
    // This ensures the PLL has stabilized at the configured frequency.
    while ((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

    // Switch the system clock source to the PLL
    // The processor will start using the 48 MHz PLL output as the main clock source.
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

    // Update the SystemCoreClock global variable to reflect the new clock frequency
    // This function adjusts the system clock value in software to match the 48 MHz setting.
    SystemCoreClockUpdate();
}

int main(int argc, char *argv[])
{
    // Configure the system clock to 48 MHz
    SystemClock48MHz();
    trace_printf("System clock: %u Hz\n", SystemCoreClock);  //Clock speed

    // Initialize GPIOA for input
    myGPIOA_Init();

    // Configure the ADC for potentiometer readings
    ADC_Config();

    // Configure the DAC to output values based on ADC input
    DAC_Config();

    // Initialize TIM2 for timer functionality (frequency measurement)
    myTIM2_Init();

    // Initialize external interrupts for EXTI0 and EXTI1 (User Button and 555 Timer)
    myEXTI_Init();

    // Configure the OLED display
    oled_config();

    // Enter an infinite loop
    while (1)
    {
        // Start the ADC conversion by setting the ADSTART bit in ADC1->CR
        ADC1->CR |= ((uint32_t)0x00000004);

        // Wait for the ADC to complete the conversion
        // The EOC (End of Conversion) flag in ADC1->ISR indicates that the conversion is complete
        while (!(ADC1->ISR & ((uint32_t)0x00000004))) {
            // Do nothing until the ADC is ready
        }

        // Retrieve the ADC converted value from the data register (DR)
        int ADC1ConvertedVal = (uint16_t)ADC1->DR;

        // Convert the ADC value to a resistance value (in ohms)
        Res = (ADC1ConvertedVal * 5000) / 0xFFF;

        // Set the DAC output to the converted ADC value
        // This outputs an analog voltage proportional to the potentiometer reading
        DAC->DHR12R1 = ADC1ConvertedVal;

        // Refresh the OLED display with the current resistance and frequency values
        refresh_OLED();
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
