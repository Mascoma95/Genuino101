/*
 * This example demonstrates using the pattern matching engine (CuriePME)
 * to classify streams of accelerometer data from CurieIMU.
 *
 * First, the sketch will prompt you to draw some letters in the air (just
 * imagine you are writing on an invisible whiteboard, using your board as the
 * pen), and the IMU data from these motions is used as training data for the
 * PME. Once training is finished, you can keep drawing letters and the PME
 * will try to guess which letter you are drawing.
 *
 * This example requires a button to be connected to digital pin 4
 * https://www.arduino.cc/en/Tutorial/Button
 *
 * NOTE: For best results, draw big letters, at least 1-2 feet tall.
 *
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See license notice at end of file.
 */

#include "CurieIMU.h"
#include "CuriePME.h"
#include <CurieSerialFlash.h>
#include <SPI.h>

#include <Adafruit_NeoPixel.h>
#define PINM 6  //// what pin are the NeoPixels connected to?
#define PINC 3
#define PINS 9
#define PINO 5 
Adafruit_NeoPixel stripM = Adafruit_NeoPixel(10, PINM, NEO_GRB + NEO_KHZ800);  /// the strip is 15 pixels long.  You can change this for the number of pixels in your individual strip.
Adafruit_NeoPixel stripS = Adafruit_NeoPixel(10, PINS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripC = Adafruit_NeoPixel(10, PINC, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripO = Adafruit_NeoPixel(10, PINO, NEO_GRB + NEO_KHZ800);

int tr = 0;  //Some variables to hold "color-target" and "color-current" for smoothing...
int tg = 0;
int tb = 0;
int r = 0;
int g = 0;
int b = 0;

int rawX = 0;  /////  to hold values from the Curie's accelerometer
int rawY = 0;
//int rawZ = 0;
float angle = 0.0;


/*  This controls how many times a letter must be drawn during training.
 *  Any higher than 4, and you may not have enough neurons for all 26 letters
 *  of the alphabet. Lower than 4 means less work for you to train a letter,
 *  but the PME may have a harder time classifying that letter. */
const unsigned int trainingReps = 4;

/* Increase this to 'A-Z' if you like-- it just takes a lot longer to train */
const unsigned char trainingStart = 'A';
const unsigned char trainingEnd = 'B'; /*'D';*/

/* The input pin used to signal when a letter is being drawn- you'll
 * need to make sure a button is attached to this pin */
const unsigned int buttonPin = 4;

/* Sample rate for accelerometer */
const unsigned int sampleRateHZ = 200;

/* No. of bytes that one neuron can hold */
const unsigned int vectorNumBytes = 128;

/* Number of processed samples (1 sample == accel x, y, z)
 * that can fit inside a neuron */
const unsigned int samplesPerVector = (vectorNumBytes / 3);

/* This value is used to convert ASCII characters A-Z
 * into decimal values 1-26, and back again. */
const unsigned int upperStart = 0x40;

const unsigned int sensorBufSize = 2048;
const int IMULow = -32768;
const int IMUHigh = 32767;

const char *filename = "NeurDataDress.dat";
const int A = 10;
const int B = 11;

void setup()
{
    Serial.begin(9600);
    //while(!Serial);

    pinMode(A,OUTPUT);
    pinMode(B,OUTPUT);
    
    pinMode(buttonPin, INPUT);

    /* Start the IMU (Intertial Measurement Unit) */
    CurieIMU.begin(ACCEL);

    /* Start the PME (Pattern Matching Engine) */
    CuriePME.begin();

    CurieIMU.setAccelerometerRate(sampleRateHZ);
    CurieIMU.setAccelerometerRange(2);

    /* Init. SPI Flash chip */
    if (!SerialFlash.begin(ONBOARD_FLASH_SPI_PORT, ONBOARD_FLASH_CS_PIN)) {
      Serial.println("Unable to access SPI Flash chip");
    }
 
    stripM.begin();  //  intialize neopixel strip
    stripS.begin();
    stripC.begin();
    stripO.begin();
    stripM.show();   // Initialize all pixels to 'off'
    stripS.show();
    stripC.show();
    stripO.show();
    solidM(stripM.Color(255, 0, 0), 50); //Red for ready for input
}

/* This function reads the file saved by the previous example
 * The file contains all the data that was learned, then saved before.
 * Once the network is restored, it is able to classify patterns again without
 * having to be retrained.*/
void restoreNetworkKnowledge ( void )
{
    SerialFlashFile file;
    int32_t fileNeuronCount = 0;

    Intel_PMT::neuronData neuronData;

    // Open the file and write test data
    file = SerialFlash.open(filename);

    CuriePME.beginRestoreMode();
    if (file) {
        // iterate over the network and save the data.
        while(1) {
            Serial.print("Reading Neuron: ");

            uint16_t neuronFields[4];
            file.read( (void*) neuronFields, 8);
            file.read( (void*) neuronData.vector, 128 );

            neuronData.context = neuronFields[0] ;
            neuronData.influence = neuronFields[1] ;
            neuronData.minInfluence = neuronFields[2] ;
            neuronData.category = neuronFields[3];

            if (neuronFields[0] == 0 || neuronFields[0] > 127)
                break;

            fileNeuronCount++;

            // this part just prints each neuron as it is restored,
            // so you can see what is happening.
            Serial.print(fileNeuronCount);
            Serial.print("\n");

            Serial.print( neuronFields[0] );
            Serial.print( "\t");
            Serial.print( neuronFields[1] );
            Serial.print( "\t");
            Serial.print( neuronFields[2] );
            Serial.print( "\t");
            Serial.print( neuronFields[3] );
            Serial.print( "\t");

            Serial.print( neuronData.vector[0] );
            Serial.print( "\t");
            Serial.print( neuronData.vector[1] );
            Serial.print( "\t");
            Serial.print( neuronData.vector[2] );
            
            Serial.print( "\n");
            CuriePME.iterateNeuronsToRestore( neuronData );
        }
    }

    CuriePME.endRestoreMode();
    Serial.print("Knowledge Set Restored. \n");
}

boolean longPress=false;
int startTime=0;
int lastOrientation = - 1; // previous orientation (for comparison)
int lastReading = -1;
boolean lastPress=false;

void loop ()
{
    int orientation = - 1;   // the board's orientation
    String orientationString; // string for printing description of orientation
  
    //  The orientations of the board:
    //  0: flat, processor facing up
    //  1: flat, processor facing down
    //  2: landscape, analog pins down
    //  3: landscape, analog pins up
    //  4: portrait, USB connector up
    //  5: portrait, USB connector down
  
    // read accelerometer:
    int x = CurieIMU.readAccelerometer(X_AXIS);
    int y = CurieIMU.readAccelerometer(Y_AXIS);
    int z = CurieIMU.readAccelerometer(Z_AXIS);

    // calculate the absolute values, to determine the largest
    int absX = abs(x);
    int absY = abs(y);
    int absZ = abs(z);

    if ( (absZ > absX) && (absZ > absY)) {
        // base orientation on Z
        if (z > 0) {
            orientationString = "up";
            orientation = 0;  
        } else {
            orientationString = "down";
            orientation = 1;
        }
    } else if ( (absY > absX) && (absY > absZ)) {
        // base orientation on Y
        if (y > 0) {
            orientationString = "digital pins up";
            orientation = 2;
        } else {
            orientationString = "analog pins up";
            orientation = 3;
        }
    } else {
      // base orientation on X
      if (x < 0) {
            orientationString = "connector up";
            orientation = 4;
      } else {
            orientationString = "connector down";
            orientation = 5;
        }
    }
    // if the orientation has changed, print out a description:
    if (orientation != lastOrientation) {
        Serial.println(orientationString);
        lastOrientation = orientation;
    }

    /*// if the orientation has changed, print out a description:
    if (orientation != lastOrientation) {
        lastOrientation = orientation;
    }
    */
    int reading = digitalRead(buttonPin);
    if (lastReading != reading) {
      Serial.print("buttonPin=");
      Serial.println(reading);
      lastReading = reading;
    }
    if (reading == HIGH) {
      if (startTime == 0) {
        startTime=millis();//Returns the number of milliseconds since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 50 days.
      } 
      else if ((millis() - startTime) >2000) {
        longPress=true;
        if (!lastPress) {
          Serial.println("longPress");
          //green for long press
          solidM(stripM.Color(0, 255, 0), 50);// Green
          lastPress=true;
        }
      }     
    }
    if ( reading == LOW && longPress ) {
        blackout(5);  
        Serial.print("orientation=");
        Serial.print(orientation);
        Serial.print(" SerialFlash.exists(filename)=");
        Serial.println(SerialFlash.exists(filename));
        if (orientation!=4 && SerialFlash.exists(filename)) {
            restoreNetworkKnowledge();
            Serial.print("Training restored.");
       } 
       else {
            trainLetters();
            blackout(5);
            Serial.print("Training complete.");
        }
        Serial.println(" Now, draw some letters (remember to ");
        Serial.println("hold the button) and see if the PME can classify them.");
        solidM(stripM.Color(0, 0, 255), 250); //solidM(stripM.Color(0, 0, 255), 500); Blue for ready for matching
        blackout(5);
        //turn off for matching
        while (true) {
            match();
        }
    }
}

void solidM(uint32_t c, uint8_t wait) {  
    for (uint16_t i=0; i < stripM.numPixels(); i++) {
        stripM.setPixelColor(i, c);        //turn every pixel off
    }
    stripM.show();
    delay(wait);
}

void blackout(uint8_t wait) {  
    for (uint16_t i=0; i < stripM.numPixels(); i++) {
        stripM.setPixelColor(i, 0);        //turn every pixel off
    }
    for (uint16_t i=0; i < stripS.numPixels(); i++) {
        stripS.setPixelColor(i, 0);        //turn every pixel off
    }
    for (uint16_t i=0; i < stripC.numPixels(); i++) {
        stripC.setPixelColor(i, 0);        //turn every pixel off
    }
    for (uint16_t i=0; i < stripO.numPixels(); i++) {
        stripO.setPixelColor(i, 0);        //turn every pixel off
    } 
    stripM.show();
    stripS.show();  
    stripC.show();
    stripO.show();
    delay(wait);
}
  
void match()
{
    byte vector[vectorNumBytes];
    unsigned int category;
    char letter;
    char pattern;
    if(digitalRead(13)==LOW){
      digitalWrite(13,HIGH);
    }
    /* Record IMU data while button is being held, and
     * convert it to a suitable vector */
    readVectorFromIMU(vector); 

    /* Use the PME to classify the vector, i.e. return a category
     * from 1-26, representing a letter from A-Z */
    category = CuriePME.classify(vector, vectorNumBytes);

    if (category == CuriePME.noMatch) {
        Serial.println("Don't recognise that one-- try again.");
        digitalWrite(A,LOW);
        digitalWrite(B,LOW); 
        //theaterChase();
        theaterChaseM(stripM.Color(127, 127, 127), 50);
        theaterChaseS(stripS.Color(127, 127, 127), 50);
        theaterChaseC(stripC.Color(127, 127, 127), 50);
        theaterChaseO(stripO.Color(127, 127, 127), 50);// White
        stripM.show();
        stripS.show();
        stripC.show();
        stripO.show();
        blackout(1);
        // delay(10);
          
    } 
    else {
        letter = category + upperStart;
        pattern = letter;
        if ( pattern == 'A' ) {
            //red
            digitalWrite(A,HIGH);
            digitalWrite(B,LOW);
            colorWipeM(stripM.Color(0, 255, 0), 50); // Green
            theaterChaseM(stripM.Color(127, 127, 127), 50); // White
            stripM.show();
            
        } else if ( pattern == 'B') {
            digitalWrite(A,LOW);
            digitalWrite(B,HIGH);
            colorWipeS(stripS.Color(255, 0, 0), 50); // Red
            theaterChaseS(stripS.Color(127, 127, 127), 50); // White
            stripS.show();
        } else if ( pattern == 'C') {
            colorWipeC(stripC.Color(0, 0, 255), 50); // Blue
            theaterChaseC(stripC.Color(127, 127, 127), 50); // White
            stripC.show();
        } else if ( pattern == 'D') {
            colorWipeO(stripO.Color(255, 0, 255), 50); // Blue
            theaterChaseO(stripO.Color(127, 127, 127), 50); // White
            stripO.show();
        }
        Serial.println(letter); 
    }
}

/* Simple "moving average" filter, removes low noise and other small
 * anomalies, with the effect of smoothing out the data stream. */
byte getAverageSample(byte samples[], unsigned int num, unsigned int pos,
                   unsigned int step)
{
    unsigned int ret;
    unsigned int size = step * 2;

    if (pos < (step * 3) || pos > (num * 3) - (step * 3)) {
        ret = samples[pos];
    } else {
        ret = 0;
        pos -= (step * 3);
        for (unsigned int i = 0; i < size; ++i) {
            ret += samples[pos - (3 * i)];
        }

        ret /= size;
    }

    return (byte)ret;
}

/* We need to compress the stream of raw accelerometer data into 128 bytes, so
 * it will fit into a neuron, while preserving as much of the original pattern
 * as possible. Assuming there will typically be 1-2 seconds worth of
 * accelerometer data at 200Hz, we will need to throw away over 90% of it to
 * meet that goal!
 *
 * This is done in 2 ways:
 *
 * 1. Each sample consists of 3 signed 16-bit values (one each for X, Y and Z).
 *    Map each 16 bit value to a range of 0-255 and pack it into a byte,
 *    cutting sample size in half.
 *
 * 2. Undersample. If we are sampling at 200Hz and the button is held for 1.2
 *    seconds, then we'll have ~240 samples. Since we know now that each
 *    sample, once compressed, will occupy 3 of our neuron's 128 bytes
 *    (see #1), then we know we can only fit 42 of those 240 samples into a
 *    single neuron (128 / 3 = 42.666). So if we take (for example) every 5th
 *    sample until we have 42, then we should cover most of the sample window
 *    and have some semblance of the original pattern. */
void undersample(byte samples[], int numSamples, byte vector[])
{
    unsigned int vi = 0;
    unsigned int si = 0;
    unsigned int step = numSamples / samplesPerVector;
    unsigned int remainder = numSamples - (step * samplesPerVector);

    /* Centre sample window */
    samples += (remainder / 2) * 3;
    for (unsigned int i = 0; i < samplesPerVector; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            vector[vi + j] = getAverageSample(samples, numSamples, si + j, step);
        }

        si += (step * 3);
        vi += 3;
    }
}

void readVectorFromIMU(byte vector[])
{
    byte accel[sensorBufSize];
    int raw[3];

    unsigned int samples = 0;
    unsigned int i = 0;

    /* Wait until button is pressed */
    while (digitalRead(buttonPin) == LOW);

    /* While button is being held... */
    while (digitalRead(buttonPin) == HIGH) {
        if (CurieIMU.dataReady()) {

            CurieIMU.readAccelerometer(raw[0], raw[1], raw[2]);

            /* Map raw values to 0-255 */
            accel[i] = (byte) map(raw[0], IMULow, IMUHigh, 0, 255);
            accel[i + 1] = (byte) map(raw[1], IMULow, IMUHigh, 0, 255);
            accel[i + 2] = (byte) map(raw[2], IMULow, IMUHigh, 0, 255);

            i += 3;
            ++samples;

            /* If there's not enough room left in the buffers
            * for the next read, then we're done */
            if (i + 3 > sensorBufSize) {
                break;
            }
        }
    }

    undersample(accel, samples, vector);
}

void trainLetter(char letter, unsigned int repeat)
{
    unsigned int i = 0;

    while (i < repeat) {
        byte vector[vectorNumBytes];
        if ( letter == 'A' ) {
            colorWipeM(stripM.Color(0, 255, 0), 50); // Green
            theaterChaseM(stripM.Color(127, 127, 127), 50); // White
            stripM.show();
        } else if ( letter == 'B') {
            colorWipeS(stripS.Color(255, 0, 0), 50); // Red
            theaterChaseS(stripS.Color(127, 127, 127), 50); // White
            stripS.show();
        } else if ( letter == 'C') {
            colorWipeC(stripC.Color(0, 0, 255), 50); // Blue
            theaterChaseC(stripC.Color(127, 127, 127), 50); // White
            stripC.show();
        } else if ( letter == 'D') {
            colorWipeO(stripO.Color(255, 0, 255), 50); // Blue
            theaterChaseO(stripO.Color(127, 127, 127), 50); // White
            stripO.show();
        }
        if (i) Serial.println("And again...");

        readVectorFromIMU(vector);
        CuriePME.learn(vector, vectorNumBytes, letter - upperStart);

        Serial.println("Got it!");
        delay(1000);
        ++i;
    }
}

bool create_if_not_exists (const char *filename, uint32_t fileSize) {
    if (!SerialFlash.exists(filename)) {
        Serial.println("Creating file " + String(filename));
        return SerialFlash.createErasable(filename, fileSize);
    }

    Serial.println("File " + String(filename) + " already exists");
    return true;
}

void saveNetworkKnowledge ( void )
{
//    const char *filename = "NeurData.dat";
    SerialFlashFile file;

    Intel_PMT::neuronData neuronData;
    uint32_t fileSize = 128 * sizeof(neuronData);

    Serial.print( "File Size to save is = ");
    Serial.print( fileSize );
    Serial.print("\n");

    create_if_not_exists( filename, fileSize );
    // Open the file and write test data
    file = SerialFlash.open(filename);
    file.erase();

    CuriePME.beginSaveMode();
    if (file) {
    // iterate over the network and save the data.
        while( uint16_t nCount = CuriePME.iterateNeuronsToSave(neuronData)) {
            if( nCount == 0x7FFF)
            break;

            Serial.print("Saving Neuron: ");
            Serial.print(nCount);
            Serial.print("\n");
            uint16_t neuronFields[4];

            neuronFields[0] = neuronData.context;
            neuronFields[1] = neuronData.influence;
            neuronFields[2] = neuronData.minInfluence;
            neuronFields[3] = neuronData.category;

            file.write( (void*) neuronFields, 8);
            file.write( (void*) neuronData.vector, 128 );
        }
    }

    CuriePME.endSaveMode();
    Serial.print("Knowledge Set Saved. \n");
}

void trainLetters()
{
    for (char i = trainingStart; i <= trainingEnd; ++i) {
        Serial.print("Hold down the button and draw the letter '");
        Serial.print(String(i) + "' in the air. Release the button as soon ");
        Serial.println("as you are done.");

        trainLetter(i, trainingReps);
        Serial.println("OK, finished with this letter.");
        delay(2000);
    }
    // Init. SPI Flash chip
    if (!SerialFlash.begin(ONBOARD_FLASH_SPI_PORT, ONBOARD_FLASH_CS_PIN)) {
      Serial.println("Unable to access SPI Flash chip");
    }
    saveNetworkKnowledge();
}

/*Special light functions from Adafruit Strandtest Example Code
 * Input a value 0 to 255 to get a color value.  Used for rainbow effect above.
 * Theater Chase lights from Adafruit strandtest example code.  This takes whatever the curent RGB value is, and does a "theatre chase" effect with it.
 */
/*Theatre-style crawling lights.*/
void theaterChasePIN(uint8_t p,uint32_t c, uint8_t wait) {
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, p, NEO_GRB + NEO_KHZ800);
    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, c);    //turn every third pixel on
            }
            strip.show();
            delay(wait);
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

/* Fill the dots one after the other with a color*/
void colorWipePIN(uint8_t p, uint32_t c, uint8_t wait) {
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, p, NEO_GRB + NEO_KHZ800);
    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);
    }
}

/*Special light functions from Adafruit Strandtest Example Code
 * Input a value 0 to 255 to get a color value.  Used for rainbow effect above.
 * Theater Chase lights from Adafruit strandtest example code.  This takes whatever the curent RGB value is, and does a "theatre chase" effect with it.
 */
void theaterChaseM(uint32_t c, uint8_t wait) {
    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < stripM.numPixels(); i=i+3) {
                stripM.setPixelColor(i+q, c);    //turn every third pixel on
            }
            stripM.show();
            delay(wait);
            for (uint16_t i=0; i < stripM.numPixels(); i=i+3) {
                stripM.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

/* Theatre-style crawling lights. */
void theaterChaseS(uint32_t c, uint8_t wait) {
    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < stripS.numPixels(); i=i+3) {
                stripS.setPixelColor(i+q, c);    //turn every third pixel on
            }
            stripS.show();
            delay(wait);
            for (uint16_t i=0; i < stripS.numPixels(); i=i+3) {
                stripS.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

/*Theatre-style crawling lights.*/
void theaterChaseC(uint32_t c, uint8_t wait) {
    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < stripC.numPixels(); i=i+3) {
                stripC.setPixelColor(i+q, c);    //turn every third pixel on
            }
            stripC.show();
            delay(wait);

            for (uint16_t i=0; i < stripC.numPixels(); i=i+3) {
                stripC.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

/*Theatre-style crawling lights.*/
void theaterChaseO(uint32_t c, uint8_t wait) {
    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < stripO.numPixels(); i=i+3) {
                stripO.setPixelColor(i+q, c);    //turn every third pixel on
            }
            stripO.show();
            delay(wait);
            for (uint16_t i=0; i < stripO.numPixels(); i=i+3) {
                stripO.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

/* Fill the dots one after the other with a color*/
void colorWipeM(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<stripM.numPixels(); i++) {
        stripM.setPixelColor(i, c);
        stripM.show();
        delay(wait);
    }
}

/* Fill the dots one after the other with a color*/
void colorWipeS(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<stripS.numPixels(); i++) {
        stripS.setPixelColor(i, c);
        stripS.show();
        delay(wait);
    }
}

/* Fill the dots one after the other with a color*/
void colorWipeC(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<stripC.numPixels(); i++) {
        stripC.setPixelColor(i, c);
        stripC.show();
        delay(wait);
    }
}

/* Fill the dots one after the other with a color*/
void colorWipeO(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<stripO.numPixels(); i++) {
        stripO.setPixelColor(i, c);
        stripO.show();
        delay(wait);
    }
}
/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
