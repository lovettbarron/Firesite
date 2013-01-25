#include "LPD8806.h"
#include "SPI.h"
#include <avr/interrupt.h> 
#include <avr/io.h> 

// Constants
const int NUMLIGHTS = 6;    // Number of lights per arm
const int NUMARMS = 1;      // Number of arms per light
const int LEDPERARM = 20;   // Number of lights
const int serialPin = 52;
const int testPin = 53;
bool connected;

int dataPins[NUMLIGHTS*NUMARMS] = {
    22,24,26,   // Light 0
    28,30,32,   // Light 1
//    34,36,38,   // Light 2
    
  //  3,5,7,   // Light 3
    //9,11,13,   // Light 4
    
    
};

int clockPins[NUMLIGHTS*NUMARMS] = {
       23,25,27,   // Light 0
    29,31,33,   // Light 1
//    35,37,39,   // Light 2

  //  2,4,6,   // Light 3
    //8,10,12,   // Light 4
    
//    41,43,45,   // Light 3
//    47,49,51,   // Light 4
//    5,7,9     // Light 5        
    
};

// Light struct
typedef struct {
    int id; // The id must match that of the OF program
    int pwr; // 0-127 val, derived from OF float val
    LPD8806 strip[NUMARMS]; // The three strips
} Light;

Light Lights[NUMLIGHTS]; // Declare lights array


// Serial communication
//String inString = ""; 
char inString[6];
int buffer;
int curLight = 0;
//int curArm = 0;
int curPwr = 0;

bool loopCounter = false;

void setup() {
    // Setup all lights and arms
    pinMode(testPin,OUTPUT);
    pinMode(serialPin,OUTPUT);
    for(int i=0;i<NUMLIGHTS;i++) {
        // Set up the Lights with the appropriate objects
        Lights[i].id = i;
        Lights[i].pwr = 0;
        for(int a=0;a<NUMARMS;a++) {
            Lights[i].strip[a] = LPD8806(LEDPERARM, dataPins[i+a],   clockPins[i+a]);   
            Lights[i].strip[a].begin();
            Lights[i].strip[a].show();
            testStrip(&Lights[i].strip[a]);
            clearStrip(&Lights[i].strip[a]);
        }
    }
    
    
    
    
    
    
    cli();    //DISABLE ALL INTERRUPTIONS
    
    //REGISTRE UBRR0
    //9600 BAUD FOSC=16MHZ
    UBRR0=103;
    
    //REGISTRE USCR0C
    //COM ASYNCHRONE
    bitWrite(UCSR0C,UMSEL00,0);
    bitWrite(UCSR0C,UMSEL01,0);
    
    //PARITY NONE
    bitWrite(UCSR0C,UPM01,0);
    bitWrite(UCSR0C,UPM00,0);
    
    //8 DATA BIT
    bitWrite(UCSR0C,UCSZ02,0);
    bitWrite(UCSR0C,UCSZ01,1);
    bitWrite(UCSR0C,UCSZ00,1);
    
    //REGISTRE UCSR0B  
    //RECEIVE & TRANSMITT ENABLE
    bitWrite(UCSR0B,RXEN0,1);
    bitWrite(UCSR0B,TXEN0,1);
    
    //ENABLE RX COMPLETE INTERRUPT
    bitWrite(UCSR0B, RXCIE0,1);
    
    sei();    //ENABLE INTERRUPTION
    
    
    // Start serial communication    
    Serial.begin(9600); connected = false;
    digitalWrite(testPin, HIGH); 

    delay(500);
}


void loop() {
    // Just turns a light on and off to viz main loop speed
    if(loopCounter) { digitalWrite(testPin, LOW); }
    else { digitalWrite(testPin, HIGH); }
    loopCounter = !loopCounter;
    
    if(connected) { // This is with a connection
        for(int i=0;i<NUMLIGHTS;i++) {
            if(Lights[i].pwr > 0) {
//                parseSerial();
                render(i);
                }
            }
        } else { // This is with no connection
            for(int i=0;i<NUMLIGHTS;i++) {
                for( int a=0;a<NUMARMS;a++) {
                   testStrip(&Lights[i].strip[a]);
                }
            }        
    }
}
              
void testStrip(LPD8806 * strip) {
    int i;
    for(i=0; i<strip->numPixels(); i++) { 
        strip->setPixelColor(i, 0);
    }
    strip->show(); 
    
    // Then display one pixel at a time:
    for(i=0; i<strip->numPixels(); i++) {
        strip->setPixelColor(i, getColor(127) ); // Set new pixel 'on'
        strip->show();              // Refresh LED states
        strip->setPixelColor(i, 0); // Erase pixel, but don't refresh!
        delay(25);
    }
    
    strip->show(); // Refresh to turn off last pixel
}

// Writes a strip accordingly
void writeStrip(LPD8806 * strip, int * pwr) {
    int i;
    for(i=0; i<strip->numPixels(); i++) { 
        strip->setPixelColor(i, 0);
    }
    strip->show(); 
    
    // Then display one pixel at a time:
    for(i=0; i<strip->numPixels(); i++) {
        strip->setPixelColor(i, getColor(*pwr) ); // Set new pixel 'on'
  //      strip->show();              // Refresh LED states
//        strip->setPixelColor(i, 0); // Erase pixel, but don't refresh!
        //delay(1);
    }
    
    strip->show(); // Refresh to turn off last pixel
}

// Sets a strip to all blank
void clearStrip(LPD8806 * strip) {
    int i;
    
    for (i=0; i < strip->numPixels(); i++) {
        strip->setPixelColor(i, getColor(0));
        strip->show();
        delay(10);
    }
}              
                                         
void render(uint32_t lightId) {
    int maxPower, minPower, ptr, ledStr, pwr;
    for(int a=0;a<NUMARMS;a++) {
        writeStrip(&Lights[lightId].strip[a], &Lights[lightId].pwr);   
    }
    
}

// Derived from the LPD8806 lib
uint32_t getColor(int pwr) {    
    byte r,g,b;
    r = (byte)1-(pwr/2);
    b = (byte)(pwr/2);
    g = (byte)0;
/*    
    r = 127;
    b = 60;
    g = 0; */
    
    return ((uint32_t)(g | 0x80) << 16) |
    ((uint32_t)(r | 0x80) <<  8) |
    b | 0x80 ;
}
                                         
void parseSerial() {
    // Read serial input:
   if(Serial.available()) {
       buffer = Serial.read();
       }
    // Collecting the datas.
    if (isDigit(buffer)) {
        //inString += (char)inChar; 
        Lights[curLight].pwr = buffer;
    }
    
    // Iterating through the lights
    if (buffer == ',') {
        Lights[curLight].pwr = buffer;
        curLight++;
    }
    
    // Last light
    if (buffer == '\n') {
        Lights[NUMLIGHTS-1].pwr = buffer;
        curLight = 0;
    }   
}

ISR(USART0_RX_vect) {  
    parseSerial();
}