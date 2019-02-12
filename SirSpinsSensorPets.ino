#include "SirSpinsSensorPets.h"

void setup() {
  #ifdef DEBUG1
  Serial.begin(115200);
  #endif
  #ifdef DEBUG2
  Serial.begin(115200);
  #endif
  // immediatly init and clear the led display
  display1.begin(0x70);  // pass in the address
  display1.clear();
  display1.writeDisplay();

  // set pin modes
  pinMode(USERBUTTON1SWITCHPIN, INPUT);
  pinMode(USERBUTTON2SWITCHPIN, INPUT);

  pinMode(USERBUTTON1LEDPIN, OUTPUT);
  digitalWrite(USERBUTTON1LEDPIN, led1on);
  pinMode(USERBUTTON2LEDPIN, OUTPUT);
  digitalWrite(USERBUTTON2LEDPIN, led2on);

//  pinMode(LEDPIN, OUTPUT);
//  digitalWrite(LEDPIN, LOW);

  pinMode(ULTRASONICTRIGPIN, OUTPUT);
  pinMode(ULTRASONICECHOPIN, INPUT);

  lidar.init();
  lidar.setTimeout(500);
  lastread = millis();
}

void loop() {  
  int idx;
  curtime = millis();

  lastdistancecm = lidar.readRangeSingleMillimeters()/10.0;
  displayInt(display1, lastdistancecm); // put the number on the led display
 
  int arrlen = sizeof(distancescm) / sizeof(distancescm[0]);
  for(idx = 0; idx < arrlen; idx++){
    if(lastdistancecm < distancescm[idx]){
      break;
      // Set your distances to be just greater than you need
    }
  }
  // note, the above may 'break' when the LIDAR reads its default no return value.  In which case the arduino will 
  // return the max 1111 value.

  byte retval = riopins[idx];
  digitalWrite(RIOPIN0, retval & 0x01);
  digitalWrite(RIOPIN1, retval >> 1 & 0x01);
  digitalWrite(RIOPIN2, retval >> 2 & 0x01);
  digitalWrite(RIOPIN3, retval >> 3 & 0x01);

}

void displayNum(Adafruit_AlphaNum4 disp, int n, uint8_t base=10){
  displayFloat(disp, (double)n, 0, 10);
}

void displayInt(Adafruit_AlphaNum4 disp, int n){
  displayNum(disp, n, 10);
}

void displayError(Adafruit_AlphaNum4 disp, char c){
  #ifdef DEBUG1
  Serial.print("In error func ");
  Serial.println(c);
  #endif
  disp.writeDigitAscii(0, 'E');
  disp.writeDigitAscii(1, 'r');
  disp.writeDigitAscii(2, 'r');
  disp.writeDigitAscii(3, c);
  disp.writeDisplay();
}

void displayFloat(Adafruit_AlphaNum4 disp, double f, uint8_t fracDigits=1, uint8_t base=10){
  // adapted from https://github.com/adafruit/Adafruit_LED_Backpack/blob/master/Adafruit_LEDBackpack.cpp
  int maxdigits = 4;
  bool isnegative = false;
  
  if(f < 0){
    // number is negative
    maxdigits--;
    isnegative = true;
    f *= -1;
  }
  
  double toIntFactor = 1.0;
  for(int i = 0; i<fracDigits; ++i) toIntFactor *= base;

  uint32_t displayNumber = f*toIntFactor+0.5;
  #ifdef DEBUG1
  Serial.print("Display Number = ");
  Serial.print(displayNumber);
  #endif

  uint32_t tooBig = 1;
  for(int i = 0; i < maxdigits; ++i) tooBig*=base;
  #ifdef DEBUG1
  Serial.print("; toobig = ");
  Serial.print(tooBig); 
  #endif

  while(displayNumber >= tooBig) {
    --fracDigits;
    toIntFactor /= base;
    displayNumber = f * toIntFactor + 0.5;
  }
  #ifdef DEBUG1
  Serial.print("; new display # = ");
  Serial.print(displayNumber);

  Serial.print("; intfac = ");
  Serial.print(toIntFactor);
  #endif

  if(toIntFactor < 1){
    displayError(disp, '1');  // Err1 - decimal beyond screen
    #ifdef DEBUG1
    Serial.print("; ERRROR");
    #endif
  } else {
    int8_t displaypos = 4;
    if(displayNumber){ // f != 0
      for(uint8_t i = 0; displayNumber || i <= maxdigits; ++i){
        boolean displayDecimal = (fracDigits != 0 && i == fracDigits);
        #ifdef DEBUG1
        Serial.print("; dd=");
        Serial.print(displayDecimal);
        #endif
        disp.writeDigitAscii(--displaypos, (displayNumber % base)+48);
        if(displayDecimal){
          disp.displaybuffer[displaypos] |= 0b0100000000000000;
        }
        displayNumber /= base;
      }
    } else {
      // f == 0
      disp.writeDigitAscii(0, '0');
      disp.writeDigitAscii(1, '0');
      disp.writeDigitAscii(2, '0');
      disp.writeDigitAscii(3, '0');
      displaypos = -1;
    }

    while(displaypos >= 0) disp.writeDigitAscii(displaypos--,' ');
    
    if(isnegative){
      disp.writeDigitAscii(0,'-');
    }
    disp.writeDisplay();
  }
  #ifdef DEBUG1
  Serial.println("");
  #endif
  
}



void ButtonCount(){
  curbutton1pressed = digitalRead(USERBUTTON1SWITCHPIN);
    
  if( curbutton1pressed == HIGH){
    if(lastbutton1pressed == false){
      presscnt += 1;
      lastbutton1pressed = true;
    }
  } else {
    lastbutton1pressed = false;
  }

  if( millis()-lastleddisplaytime > ledupdatetime){
    digitalWrite(USERBUTTON1LEDPIN, led1on);
    digitalWrite(USERBUTTON2LEDPIN, led2on);
//    digitalWrite(LEDPIN, led1on);

    displayFloat(display1, presscnt, 0, 10);

    lastleddisplaytime = curtime;
    
  }
}

void displayString(Adafruit_AlphaNum4 disp, String str, bool imexit = false){
  for(int idx = 0; idx < str.length()-3; idx++){
    disp.writeDigitAscii(0, str[idx]);
    disp.writeDigitAscii(1, str[idx+1]);
    disp.writeDigitAscii(2, str[idx+2]);
    disp.writeDigitAscii(3, str[idx+3]);
    disp.writeDisplay();
    
    if(imexit) return;
    
    int brightness;
    if(idx == 0){
      brightness = 255; 
      delay(scrolldelay/2); // extra startup delay
      brightness -= 5;
      if(brightness < 24) brightness = 24;
      analogWrite(USERBUTTON2LEDPIN, char(brightness) );
      delay(scrolldelay/2);
    }
    brightness = 255-(idx+1)*10;
    if(brightness < 24) brightness = 24;
    analogWrite(USERBUTTON2LEDPIN, char(brightness) );
    delay(scrolldelay/2);
    brightness -= 5;
    if(brightness < 24) brightness = 24;
    analogWrite(USERBUTTON2LEDPIN, char(brightness) );
    delay(scrolldelay/2);
  }
}

void IRProx(){
  digitalWrite(USERBUTTON1LEDPIN, !digitalRead(IRDDSPIN));
}

void Ultrasonic(){
  // trigger the ping
  digitalWrite(ULTRASONICTRIGPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRASONICTRIGPIN, HIGH);
  digitalWrite(USERBUTTON1LEDPIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(ULTRASONICTRIGPIN, LOW);
  duration = pulseIn(ULTRASONICECHOPIN, HIGH, 30000);
  distancecm = duration / 58.2;  // 29.1*2
  displayFloat(display1, distancecm);
  digitalWrite(USERBUTTON1LEDPIN, LOW);
  delay(100);
}

void UltrasonicWAvg(){
  digitalWrite(USERBUTTON1LEDPIN, HIGH);
  distancecm = sonar.convert_cm(sonar.ping_median(5));
  displayFloat(display1, distancecm);
  digitalWrite(USERBUTTON1LEDPIN, LOW);
  delay(100);
}

void readAnalog1(){
  displayFloat(display1, analogRead(ANALOG1PIN), 0);
}

void readLidar(){
  //digitalWrite(USERBUTTON1LEDPIN, HIGH);
  //displayFloat(display1, lidar.readRangeSingleMillimeters()/10.0);
  //digitalWrite(USERBUTTON1LEDPIN, LOW);
  //delay(100);
  lastdistancemm = lidar.readRangeSingleMillimeters();
}

/* 
 * Silly arduino and their limited spi library
 * See http://www.gammon.com.au/forum/?id=10892
 */
ISR(SPI_STC_vect){
  if(pos < 10){
    SPDR = spibuf[pos];
    pos++;
  }
}
