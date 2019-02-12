#include <Wire.h>
#include <Adafruit_GFX.h>
#include <NewPing.h>
#include <VL53L0X.h>
#include "Adafruit_LEDBackpack.h"

#define DEBUG2

Adafruit_AlphaNum4 display1 = Adafruit_AlphaNum4();

#define USERBUTTON1SWITCHPIN 3
#define USERBUTTON2SWITCHPIN A2
#define USERBUTTON1LEDPIN 5
#define USERBUTTON2LEDPIN 6
#define ULTRASONICTRIGPIN 2
#define ULTRASONICECHOPIN 4
#define ANALOG1PIN A0
#define ANALOG2PIN A1
#define IRDDSPIN 9
//#define LEDPIN 13

#define RIOPIN0 10
#define RIOPIN1 11
#define RIOPIN2 12
#define RIOPIN3 13

//#define NUMMODES 6
//#define MODEButtonCnt 0
//#define MODEAnalog1 1
//#define MODEUltrasonic 2
//#define MODEUltrasonicAVG 3
//#define MODELIDAR 4
//#define MODEIRDDS 5

int distancescm[] = {
  -1, // 0000
  2,  // 0001
  4,  // 0010
  6,  // 0011
  8,  // 0100
  10, // 0101
  12, // 0110
  14, // 0111
  16, // 1000
  18, // 1001
  20, // 1010
  22, // 1011
  24, // 1100
  26, // 1101
  28, // 1110
  30  // 1111
};

byte riopins[] = {
  0b00000000, // -1
  0b00000001, // 2
  0b00000010, // 4
  0b00000011, // 6
  0b00000100, // 8
  0b00000101, // 10
  0b00000110, // 12
  0b00000111, // 14
  0b00001000, // 16
  0b00001001, // 18
  0b00001010, // 20
  0b00001011, // 22
  0b00001100, // 24
  0b00001101, // 26
  0b00001110, // 28
  0b00001111  // 30
};

/*
char * modestrings[] = {
  "PRESS BUTTON EXACTLY ONCE",
  "READ ANALOG INPUT ONE",
  "READ ULTRASONIC - SINGLE READS",
  "READ ULTRASONIC - WITH AVERAGING",
  "READ LIDAR",
  "READ IR ON/OFF PROXIMITY"   
};
*/
/*
bool switchingmodes = true;
unsigned long modesettime = 0;
*/

int lastdistancemm;
int lastdistancecm;

long lastread;
long readspacing = 50;

//uint8_t operatingmode = MODELIDAR;

bool led1on = false;
bool led2on = false;

char spibuf [100];
volatile byte pos;
volatile boolean newdata;

unsigned long curtime = 0;
unsigned long lastleddisplaytime = 0;
static unsigned int ledupdatetime = 50;
static unsigned int buttondebouncetime = 50;
static int scrolldelay = 200;

char display1buffer[4] = {' ', ' ', ' ', ' '};

bool lastbutton1pressed = false;
bool curbutton1pressed = false;
bool lastbutton2pressed = false;
bool curbutton2pressed = false;

unsigned long button2presstime;

uint8_t modecharpos = 0;

// mode Button Count Variables
unsigned long presscnt = 0;

// mode Ultrasonic Variables
NewPing sonar(ULTRASONICTRIGPIN, ULTRASONICECHOPIN, 400);
unsigned long duration; 
float distancecm;
// mode Lidar Variables
VL53L0X lidar;

// function headers
void setup();
void displayNum(Adafruit_AlphaNum4, int, uint8_t);
void displayInt(Adafruit_AlphaNum4, int);
void displayError(Adafruit_AlphaNum4, char);
void displayFloat(Adafruit_AlphaNum4 disp, double f, uint8_t fracDigits=1, uint8_t base=10);
void loop();
void ButtonCount();
void IRProx();
void displayString(Adafruit_AlphaNum4, String, bool imexit = false);
void Ultrasonic();
void UltrasonicWAvg();
void readAnalog1();
void readLidar();
