/* Jake Wimberley, K4ZAU, June 2015
   Code based on the RebelAllianceMod v1.1, which is itself based on the original Ten-Tec code.
   
   TX is turned off in all User modes.
   
   U1 = Query mode.
        Any key touch announces freq.
        "Select" LEDs flash, and indicate S meter strength in binary.
          S value is the displayed bits plus one, so the max value is S9.
          Least significant bit is bottom.
          So if the red and green are lit, (1 + 3) + 1 = S5.
          No LEDs means signal is below S2.
   U2 = Keyer speed.
        Turn dial to change.
        Sidetone "I" on every step, to indicate speed.
   U3 = VFO "A/B"
        Both are initialized to the default freq.
        Dit paddle = Swap VFOs.
        Dah paddle = Set VFOs equal.
   
*/

// jcw options
#undef DEBUG
#undef NOTX

/*

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
 
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
   
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  */

  /*
    Prog for ad9834
    Serial timming setup for AD9834 DDS
    start > Fsync is high (1), Sclk taken high (1), Data is stable (0, or 1),
    Fsync is taken low (0), Sclk is taken low (0), then high (1), data changes
    Sclk starts again.
    Control Register D15, D14 = 00, D13(B28) = 1, D12(HLB) = X,
    Reset goes high to set the internal reg to 0 and sets the output to midscale.
    Reset is then taken low to enable output. 
   ***************************************************/   

  // various defines
#define SDATA_BIT                           10          //  keep!
#define SCLK_BIT                            8           //  keep!
#define FSYNC_BIT                           9           //  keep!
#define RESET_BIT                           11          //  keep!
#define FREQ_REGISTER_BIT                   12          //  keep!
#define AD9834_FREQ0_REGISTER_SELECT_BIT    0x4000      //  keep!
#define AD9834_FREQ1_REGISTER_SELECT_BIT    0x8000      //  keep!
#define FREQ0_INIT_VALUE                    0x01320000  //  ?

#define led                                 13          // Ten*Tec led
#define Side_Tone                           3           // maybe to be changed to a logic control
                                                          // for a separate side tone gen
#define Macro_TX_Dah                        33          //  keep!
#define Macro_TX_Dit                        32          //  keep!
#define TX_OUT                              38          //  keep!

#define Band_End_Flash_led                  24          // // also this led will flash every 100/1khz/10khz is tuned
#define Band_Select                         41          // if shorting block on only one pin 20m(1) on both pins 40m(0)
#define Multi_Function_Button               2           //
#define Multi_function_Green                34          // For now assigned to BW (Band width)
#define Multi_function_Yellow               35          // For now assigned to STEP size
#define Multi_function_Red                  36          // For now assigned to USER

#define Select_Button                       5           // 
#define Select_Green                        37          // Wide/100/USER1
#define Select_Yellow                       39          // Medium/1K/USER2
#define Select_Red                          40          // Narrow/10K/USER3

#define Medium_A8                           22          // Hardware control of I.F. filter Bandwidth
#define Narrow_A9                           23          // Hardware control of I.F. filter Bandwidth

#define Wide_BW                             0           // About 2.1 KHZ
#define Medium_BW                           1           // About 1.7 KHZ
#define Narrow_BW                           2           // About 1 KHZ

#define Step_100_Hz                         0
#define Step_1000_hz                        1
#define Step_10000_hz                       2

#define  Other_1_user                       0           // 
#define  Other_2_user                       1           //
#define  Other_3_user                       2           //

  // various defines continue
  const int RitReadPin        = A0;      // pin that the sensor is attached to used for a rit routine later.
  int RitReadValue            = 0;
  int RitFreqOffset           = 0;

  const int SMeterReadPin     = A1;      // To give a realitive signal strength based on AGC voltage.
  float SMeterReadValue       = 0;

  const int BatteryReadPin    = A2;      // Reads 1/5 th or 0.20 of supply voltage.
  float BatteryReadValue      = 0;
  float BatteryVconvert       = 0.01707; //callibrated on 13.8v ps

  const int PowerOutReadPin   = A3;      // Reads RF out voltage at Antenna.
  int PowerOutReadValue       = 0;

  const int CodeReadPin       = A6;      // Can be used to decode CW. 
  int CodeReadValue           = 0;

  const int InternalPotPin    = A7;      // To adjust CW speed for user written keyer.
  int InternalPotValue        = 0;            
  unsigned long ditTime;                 // No. milliseconds per dit

// jcw: allow software swapping of paddle lines
  int TX_Dah = Macro_TX_Dah;
  int TX_Dit = Macro_TX_Dit;

  int ST_key = 0;        //This variable tells TX routine whether to enter use straight key mode

#ifdef DEBUG
    void    serialDump();
#endif

  // various variables
  int TX_key;
  int band_sel;                               // select band 40 or 20 meter
  int band_set;
  int bsm                         = 0;  
  int Step_Select_Button          = 0;
  int Step_Select_Button1         = 0;
  int Step_Multi_Function_Button  = 0;
  int Step_Multi_Function_Button1 = 0;
  int Selected_BW                 = 0;    // current Band width 
                                          // 0= wide, 1 = medium, 2= narrow
  int Selected_Step               = 0;    // Current Step
  int Selected_Other              = 0;    // To be used for anything

  // jcw additions
  int CWSpeed = 13;
  int CWSpeed_Minimum = 5;
  int CWSpeed_Maximum = 40;
  float CWWeight = 2.75;
  int SMeter_Running = 0; // if 1, S Meter is enabled
  int SMeter_Displayed = 0; // when 0, S Meter LEDs are flashed "off"

  // Encoder Stuff 
  const int encoder0PinA          = 7;
  const int encoder0PinB          = 6;
  int val; 
  int encoder0Pos                 = 0;
  int encoder0PinALast            = LOW;
  int n                           = LOW;

  // frequency vaiables and memory
  const long meter_40             = 16.03e6;      // IF + Band frequency, 
  long meter_40_memory            = 16.03e6;      // HI side injection 40 meter 
                                                  // range 16 > 16.3 mhz
  const long meter_20             = 5.06e6;       // Band frequency - IF, LOW 
  long meter_20_memory            = 5.06e6;       // side injection 20 meter 
                                                  // range 5 > 5.35 mhz
  const long Reference            = 49.99975e6;   // for ad9834 this may be 
                                                  // tweaked in software to 
                                                  // fine tune the Radio
  long RIT_frequency;
  long RX_frequency;
  long TX_frequency;
  long save_rec_frequency;
  long frequency_step;
  long *frequency;
  long frequency_old              = 0;
  long frequency_tune             = 0;
  long frequencyA = 0;
  long frequencyB = 0;
  long frequency_scratch = 0;
  long frequency_default          = 0;
  long fcalc;
  long IF                         = 9.00e6;          //  I.F. Frequency

  // Timer variables for Debug and Display Refresh 
  unsigned long loopCount         = 0;
  unsigned long lastLoopCount     = 0;
  unsigned long loopsPerSecond    = 0;
  unsigned int  printCount        = 0;

  unsigned long loopStartTime     = 0;
  unsigned long loopElapsedTime   = 0;
  float         loopSpeed         = 0;
  unsigned long LastFreqWriteTime = 0;

  // For timing stabilization
  unsigned long loopUsecStart = 0;
  unsigned long loopUsecLength = 0; // 0 to disable

  unsigned long morseStartTime = 0;
  unsigned long morseElapsedTime = 0;
  unsigned long morseElementTime = 0;
  bool morseIntraCharacterPause = false;
  char sidetoneBuffer[1024];
  int sidetoneIndex = -1;
  unsigned int sidetoneBufferLength = 0;

  // for debouncing
  unsigned long lastKeyCheck = 0;
  unsigned int keyDebounceDelay = 10;

  unsigned long keyerStartTime = 0;
  unsigned long keyerElapsedTime = 0;
  unsigned long keyerElementTime = 0;
  bool keyerPause = false;
  bool latchDit = false;
  bool latchDah = false;
  
  int keyRead_TaskId;
  unsigned long keyRead_TaskVar;

  //Program routines@
  void Default_frequency();
  void AD9834_init();
  void AD9834_reset();
  void program_freq0(long freq);
  void program_freq1(long freq1);  // added 1 to freq
  void UpdateFreq(long freq);
  void led_on_off();
  void Frequency_up();                        
  void Frequency_down();                      
  void TX_routine();
  void RX_routine();
  void Encoder();
  void AD9834_reset_low();
  void AD9834_reset_high();
  void Band_Set_40M_20M();
  void Band_40M_limits_led();
  void Band_20M_limits_led();
  void Step_Flash();
  void RIT_Read();
  void Multi_Function();          //
  void Step_Selection();          // 
  void Selection();               //
  void Step_Multi_Function();     //

  // jcw
  void SMeter();
  void Keyer_AdjustWPM();
  void AnnounceFrequency(bool);

  void MF_G();                    // Controls Function Green led
  void MF_Y();                    // Controls Function Yellow led
  void MF_R();                    // Controls Function Red led

  void S_G();                     // Controls Selection Green led & 
                                  // Band_Width wide, Step_Size 100, Other_1

  void S_Y();                     // Controls Selection Green led & 
                                  // Band_Width medium, Step_Size 1k, Other_2

  void S_R();                     // Controls Selection Green led & 
                                  // Band_Width narrow, Step_Size 10k, Other_3

  void Band_Width_W();            //  A8+A9 low
  void Band_Width_M();            //  A8 high, A9 low
  void Band_Width_N();            //  A8 low, A9 high

  void Step_Size_100();           //   100 hz step
  void Step_Size_1k();            //   1 kilo-hz step
  void Step_Size_10k();           //   10 kilo-hz step

  void Other_1();                 //   user 1
  void Other_2();                 //   user 2
  void Other_3();                 //   user 3 

  void clock_data_to_ad9834(unsigned int data_word);

  // Setup and initialize 
  void setup() 
  {
    // these pins are for the AD9834 control
    pinMode(SCLK_BIT,               OUTPUT);    // clock
    pinMode(FSYNC_BIT,              OUTPUT);    // fsync
    pinMode(SDATA_BIT,              OUTPUT);    // data
    pinMode(RESET_BIT,              OUTPUT);    // reset
    pinMode(FREQ_REGISTER_BIT,      OUTPUT);    // freq register select

    //---------------  Encoder ------------------------------------
    pinMode (encoder0PinA,          INPUT);     // using optical for now
    pinMode (encoder0PinB,          INPUT);     // using optical for now 

    //---------------  Keyer --------------------------------------
    pinMode (TX_Dit,                INPUT);     // Dit Key line 
    pinMode (TX_Dah,                INPUT);     // Dah Key line
    pinMode (TX_OUT,                OUTPUT);
    pinMode (Band_End_Flash_led,    OUTPUT);
      
    //---------------- Menu leds ----------------------------------
    pinMode (Multi_function_Green,  OUTPUT);    // Band width
    pinMode (Multi_function_Yellow, OUTPUT);    // Step size
    pinMode (Multi_function_Red,    OUTPUT);    // Other
    pinMode (Multi_Function_Button, INPUT);     // Choose from Band width, Step size, Other

    //----------------- Selection leds ----------------------------
    pinMode (Select_Green,          OUTPUT);    //  BW wide, 100 hz step, other1
    pinMode (Select_Yellow,         OUTPUT);    //  BW medium, 1 khz step, other2
    pinMode (Select_Red,            OUTPUT);    //  BW narrow, 10 khz step, other3
    pinMode (Select_Button,         INPUT);     //  Selection from the above

    pinMode (Medium_A8,             OUTPUT);    // Hardware control of I.F. filter Bandwidth
    pinMode (Narrow_A9,             OUTPUT);    // Hardware control of I.F. filter Bandwidth
      
    pinMode (Side_Tone,             OUTPUT);    // sidetone enable
    
    pinMode (Band_Select,           INPUT);

    
    Default_Settings();

    AD9834_init();
    AD9834_reset();                               // low to high
    
    // initialize VFOs to default freq individually
    frequency = &frequencyB;                      // must initialize pointer to a VFO
    Band_Set_40_20M();                            // read jumper setting and adjust freq accordingly
    frequency = &frequencyA;
    Default_frequency();                          // this leaves "A" as the one tuned first by the user
   
    digitalWrite(TX_OUT,            LOW);         // turn off TX

    //--------------------------------------------------------------
    Step_Size_100();                              // Change for other Step_Size default!
    for (int i=0; i <= 5e4; i++);                 // small delay

    AD9834_init();
    AD9834_reset();

    encoder0PinALast = digitalRead(encoder0PinA);  
    //attachInterrupt(encoder0PinA, Encoder, CHANGE);
    //attachInterrupt(encoder0PinB, Encoder, CHANGE);
    attachCoreTimerService(TimerOverFlow);//See function at the bottom of the file.

#ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Rebel Ready:");
#endif
    
    loadWPM(CWSpeed);                                // Set default CW Speed 
    //See if user wants to use a straight key
    if ((digitalRead(TX_Dah) == LOW) || (digitalRead(TX_Dit) == LOW)) {    //Is a lever pressed?
      ST_key = 1;      //If so, enter straight key mode   
    }
    
    // Reverse paddle if SELECT button is held at start-up
    if (digitalRead(Select_Button) == HIGH) {
      int swap;
      swap = TX_Dah;
      TX_Dah = TX_Dit;
      TX_Dit = swap;
    }
    
    keyRead_TaskId = createTask(keyRead_Task, 1, TASK_ENABLE, &keyRead_TaskVar);
    
    delay(1000); // allow release of straight key before entering main loop

  }   //    end of setup


  // Default settings 
  void Default_Settings()
  {
    digitalWrite(Multi_function_Green,  LOW);    // Band_Width
                                                 // place control here

    digitalWrite(Multi_function_Yellow, LOW);    //
                                                 // place control here

    digitalWrite(Multi_function_Red,    HIGH);   //
                                                 // place control here

    digitalWrite(Select_Green,          HIGH);   //  
    Band_Width_N();                              // place control here 

    digitalWrite(Select_Yellow,         LOW);    //
                                                 // place control here

    digitalWrite(Select_Green,          LOW);    //
                                                 // place control here
    digitalWrite(TX_OUT,                LOW);   

    digitalWrite(FREQ_REGISTER_BIT,     LOW);    //This is set to LOW so RX is not dead on power on        
                                                  
    digitalWrite(Band_End_Flash_led,    LOW);

    digitalWrite(Side_Tone,             LOW);    
  }

  //======================= Main Loop =================================
  void loop()      
  {

    loopUsecStart = micros();

    digitalWrite(FSYNC_BIT,             HIGH);   // 
    digitalWrite(SCLK_BIT,              HIGH);   //

    // Check value of RIT knob. RitFreqOffset will be set to the offset in Hz
    RIT_Read();

    // Act if FUNCTION is pressed.
    Multi_Function(); 

    Encoder();

    // If user has selected User mode and the setting is "green", announce frequency on key press
    // jcw: if if() doesn't work, try while()
    
    // TODO START HERE need debouncing for key presses
    if ( Selected_Other == 0 && Step_Multi_Function_Button1 == 2 )
    {
        if (sidetoneIndex < 0) {
          if (digitalRead(TX_Dah) == LOW) AnnounceFrequency(false);
          else if (digitalRead(TX_Dit) == LOW) AnnounceFrequency(true);
        }
        SMeter_Running = 1;
        Multi_Function();       //Must run MultiFunction code to detect when user exits either User mode (right button) or user option 2 (left button)
    } else {
        SMeter_Running = 0;
    }
    if ( Selected_Other == 2 && Step_Multi_Function_Button1 == 2 ) {
      if (digitalRead(TX_Dit) == LOW) { // VFO swap
        if (frequency == &frequencyA) {
          frequency = &frequencyB;
          strcpy(sidetoneBuffer,"-...|");
          sidetoneIndex = 0;
        } else {
          frequency = &frequencyA;
          strcpy(sidetoneBuffer,".-|");
          sidetoneIndex = 0;
        }
      } else if (digitalRead(TX_Dah) == LOW) { // set VFOs equal
        // the "idle" VFO gets set to the value of the "active" one
        if (frequency == &frequencyA)
          frequencyB = *frequency;
        else
          frequencyA = *frequency;
      }
    }
    
    SMeter();

    frequency_tune  = (*frequency) + RitFreqOffset;
    UpdateFreq(frequency_tune);
   
    //TX_routine();
      
    loopCount++;
    loopElapsedTime    = millis() - loopStartTime;    // comment this out to remove the one second tick

    // has 1000 milliseconds elasped?
    if( 1000 <= loopElapsedTime )
    { 
#ifdef DEBUG
        serialDump();
#endif
        
        SMeter_Displayed ^= 1;

        loopStartTime   = millis();
    }

    /* Morse timing: Controls both transmission and sidetone buffer.
       At start of dit or dah, either for transmit or sidetone, start a timer.
       When ditTime (a function of wpm and weight) has passed, step sequence;
         that is, clear timer, end one action and start another (or go idle).
       After a dit or dah ends, if the next element in the buffer is
         a dit or dah, insert a pause for the length of a dit.
       The cleanest way to insert the pause was to have a separate timing
         mechanism.
    */

    if (morseIntraCharacterPause) {
      // This effectively delays action on the next element.
      if (morseElapsedTime == 0) morseStartTime = millis();
      for (int zzz = 0; zzz < 1e4; zzz++); // tiny delay to guarantee morseElapsedTime is greater than 0 on first check
      morseElapsedTime = millis() - morseStartTime;
      if (morseElapsedTime >= ditTime) {
        morseElapsedTime = 0;
        morseIntraCharacterPause = false;
      }
    }
    else if (sidetoneIndex >= 0) {
      if (morseElapsedTime == 0) {
        morseStartTime = millis();
        // start element
        char elem = sidetoneBuffer[sidetoneIndex];
        switch (elem) {
          case '.':
            morseElementTime = ditTime;
            digitalWrite(Side_Tone, HIGH);     // Start tone
            break;
          case '-':
            morseElementTime = ditTime * CWWeight;
            digitalWrite(Side_Tone, HIGH);     // Start tone
            break;
          case '|':
            morseElementTime = ditTime * CWWeight;
            digitalWrite(Side_Tone, LOW);     // No tone
            break;
          case ' ':
            morseElementTime = ditTime * 7;
            digitalWrite(Side_Tone, LOW);     // No tone
            break;
        }
      }
      morseElapsedTime = millis() - morseStartTime;
      if (morseElapsedTime >= morseElementTime) {
        // stop element
        digitalWrite(Side_Tone, LOW);
        morseElapsedTime = 0;
        sidetoneIndex++;
        if (sidetoneIndex > strlen(sidetoneBuffer)) { // end of buffer, stop sidetone sequence
          sidetoneIndex = -1;
          //memset(sidetoneBuffer,'\0',1024); //TODO don't do, want to send stuff added to buffer during cycle
        }
        // insert intra-character space if new buffer element is a sound
        if (sidetoneBuffer[sidetoneIndex] == '.' || sidetoneBuffer[sidetoneIndex] == '-')
          morseIntraCharacterPause = true;
      } // end if morseElapsedTime >= morseElementTime
    } // end if sidetoneIndex >= 0

    if (ST_key) {
      if (millis() - lastKeyCheck > keyDebounceDelay) {
        if (digitalRead(TX_Dah) == LOW || digitalRead(TX_Dit) == LOW) {
          digitalWrite(Side_Tone, HIGH);
#ifndef NOTX
          // transmit only if user mode is not selected
          if (Step_Multi_Function_Button1 != 2) {
            digitalWrite(TX_OUT, HIGH);
            digitalWrite(FREQ_REGISTER_BIT, HIGH);
          } 
#endif
        } else {
          digitalWrite(Side_Tone, LOW);
          digitalWrite(TX_OUT, LOW);
#ifndef NOTX
          for (int i=0; i <= 10e3; i++); // delay for maybe some decay on key release
          digitalWrite(FREQ_REGISTER_BIT, LOW);
#endif
        }
        lastKeyCheck = millis();
      }
    } else {
      // When a lever is pressed, initiate a sequence of an element as well as a dit space.
      if (keyerPause) {
        // insert pause before starting cycle again
        if (keyerElapsedTime == 0) keyerStartTime = millis();
        for (int zzz = 0; zzz < 1e4; zzz++); // tiny delay to guarantee keyerElapsedTime is greater than 0 on first check
        keyerElapsedTime = millis() - keyerStartTime;
        if (keyerElapsedTime >= ditTime) {
          keyerElapsedTime = 0;
          keyerPause = false;
        }
        if (digitalRead(TX_Dit) == LOW) latchDit = true;
        if (digitalRead(TX_Dah) == LOW) latchDah = true;
      }
      if (keyerElapsedTime == 0) {
        keyerStartTime = millis();
        if (digitalRead(TX_Dit) == LOW || latchDit) {
          keyerElementTime = ditTime;
          digitalWrite(Side_Tone, HIGH);     // Start tone
#ifndef NOTX
          // transmit only if user mode is not selected
          if (Step_Multi_Function_Button1 != 2) {
            digitalWrite(TX_OUT, HIGH);
            digitalWrite(FREQ_REGISTER_BIT, HIGH);
          }
#endif
          latchDit = false;
        } else if (digitalRead(TX_Dah) == LOW || latchDah) {
          keyerElementTime = ditTime * CWWeight;
          digitalWrite(Side_Tone, HIGH);     // Start tone
#ifndef NOTX
          // transmit only if user mode is not selected
          if (Step_Multi_Function_Button1 != 2) {
            digitalWrite(TX_OUT, HIGH);
            digitalWrite(FREQ_REGISTER_BIT, HIGH);
          }
#endif
          latchDah = false;
        }
      }
      if (keyerElementTime) {
        if (digitalRead(TX_Dit) == LOW && keyerElementTime == ditTime * CWWeight) latchDit = true;
        if (digitalRead(TX_Dah) == LOW && keyerElementTime == ditTime) latchDah = true;
        keyerElapsedTime = millis() - keyerStartTime;
        if (keyerElapsedTime >= keyerElementTime) {
          // stop element
          digitalWrite(Side_Tone, LOW);
          digitalWrite(TX_OUT, LOW);
#ifndef NOTX
          for (int i=0; i <= 10e3; i++); // delay for maybe some decay on key release
          digitalWrite(FREQ_REGISTER_BIT, LOW);
#endif
          keyerElapsedTime = 0;
          keyerElementTime = 0;
          keyerPause = true;
        }
      }
    } 

    // timing correction
    unsigned int loopElapsedUsec = micros() - loopUsecStart;
    if (loopElapsedUsec < loopUsecLength)
      delayMicroseconds(loopUsecLength - loopElapsedUsec);

}    //  END LOOP
//===================================================================

void keyRead_Task(int id, void *tptr) {
            // latch lever if pressed
          if (digitalRead(TX_Dit) == LOW && keyerElementTime == ditTime * CWWeight) latchDit = true;
          if (digitalRead(TX_Dah) == LOW && keyerElementTime == ditTime) latchDah = true;
}

//------------------ Debug data output ------------------------------
#ifdef DEBUG
void    serialDump()
{
  loopsPerSecond  = loopCount - lastLoopCount;
  loopSpeed       = (float)1e6 / loopsPerSecond;
  lastLoopCount   = loopCount;

  Serial.print    ( "uptime: " );
  Serial.print    ( ++printCount );
  Serial.println  ( " seconds" );

  Serial.print    ( "loops per second:    " );
  Serial.println  ( loopsPerSecond );
  Serial.print    ( "loop execution time: " );
  Serial.print    ( loopSpeed, 3 );
  Serial.println  ( " uS" );

  Serial.print("VFO A = "); Serial.println(frequencyA);
  Serial.print("VFO B = "); Serial.println(frequencyB);
  Serial.print    ( "Freq Rx: " );
  Serial.println  ( frequency_tune + IF );
  Serial.print    ( "Freq Tx: " );
  Serial.println  ( (*frequency) + IF );
    
  Serial.print    ( "CW speed:" );
  Serial.println  ( CWSpeed );
  Serial.println  ();
 
  Serial.print    ( "AGC voltage:" );
  Serial.println  ( analogRead(SMeterReadPin) );
  Serial.println  ();
  
  Serial.print("ditTime: ");
  Serial.println(ditTime);

  Serial.print    ( "Sidetone index:" );
  Serial.println  ( sidetoneIndex );
  Serial.print    ( "Sidetone buffer:" );
  Serial.println  ( sidetoneBuffer );
} // end serialDump()
#endif


//------------------ Band Select ------------------------------------
void Band_Set_40_20M()
{
    bsm = digitalRead(Band_Select); 
    //  select 40 or 20 meters 1 for 20 0 for 40
    if ( bsm == 1 ) 
    { 
        frequency_default = meter_20;
    }
    else 
    { 
        frequency_default = meter_40; 
        IF *= -1;               //  HI side injection
    }

    Default_frequency();
}

//--------------------------- Encoder Routine ----------------------------  
void Encoder()
{  
    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH)) 
    {
        if (digitalRead(encoder0PinB) == LOW) 
        {
            // If User option 2 is selected and no Straight Key is detected, change keyer speed.
            if ( Selected_Other == 1 && Step_Multi_Function_Button1 == 2 && ST_key == 0)
            {
                Keyer_AdjustWPM(-1);
                Multi_Function(); //Must run MultiFunction() to detect when user presses either button
            } else {
                Frequency_down();    //encoder0Pos--;
            }
        } else 
        {
            // If User option 2 is selected and no Straight Key is detected, change keyer speed.
            if ( Selected_Other == 1 && Step_Multi_Function_Button1 == 2 && ST_key == 0)
            {
                Keyer_AdjustWPM(1);
                Multi_Function(); //Must run MultiFunction() to detect when user presses either button
            } else {
                Frequency_up();       //encoder0Pos++;
            }
        }
    } 
    encoder0PinALast = n;
}
//----------------------------------------------------------------------
void Frequency_up()
{ 
    *frequency = (*frequency) + frequency_step;
    
    Step_Flash();
    

}

//------------------------------------------------------------------------------  
void Frequency_down()
{ 
    *frequency = (*frequency) - frequency_step;
    
    Step_Flash();
    
}
//-------------------------------------------------------------------------------
void UpdateFreq(long freq)
{
    long freq1;
    //  some of this code affects the way to Rit responds to being turned
    if (LastFreqWriteTime != 0)
    { if ((millis() - LastFreqWriteTime) < 100) return; }
    LastFreqWriteTime = millis();

    if(freq == frequency_old) return;

    program_freq0( freq  );
    
    freq1 = freq - RitFreqOffset;  //  to get the TX freq

    program_freq1( freq1 + IF  );
  
    frequency_old = freq;
}

// Calculate new time constants based on wpm value
void loadWPM(int wpm)
{
    // www.kent-engineers.com/codespeed.htm
    ditTime = 60000 / (wpm * (26 + (CWWeight * 8)));
    //ditTime = 1200/(wpm+3);              // correction factor = 3
}
// Checks the Keyer speed Pot and updates value
/* void checkWPM() 
{
   InternalPotValue = analogRead(InternalPotPin);
   InternalPotValue = map(InternalPotValue, 0, 1024, 5, 45);
   loadWPM(InternalPotValue);
} */

void Keyer_AdjustWPM(int increment)                                
{
    CWSpeed += increment;
    if (CWSpeed < CWSpeed_Minimum) CWSpeed = CWSpeed_Minimum;
    if (CWSpeed > CWSpeed_Maximum) CWSpeed = CWSpeed_Maximum;
    loadWPM(CWSpeed);                  // Update key speed
    // don't activate sidetone unless buffer is empty
    if (sidetoneIndex < 0) {
      strcpy(sidetoneBuffer,"...|");
      sidetoneIndex = 0;
    }
    /*
    digitalWrite(Side_Tone, HIGH);     // Start tone for the first dit in 'I'
    delay(ditTime);                    // Continue tone for the right amount of time
    digitalWrite(Side_Tone, LOW);      // Stop tone
    delay(ditTime * CWWeight);         // gap between dits
    digitalWrite(Side_Tone, HIGH);     // Start tone for the second dit in 'I'
    delay(ditTime);                    // Continue tone for the right amount of time
    digitalWrite(Side_Tone, LOW);      // Stop tone
    */
}

// S-Meter
void SMeter() {
  return;
}

// Frequency announcer
void AnnounceFrequency(bool bRx) {
  int actualFreq;
  if (bRx) {
    actualFreq = frequency_tune + IF;
  } else {
    actualFreq = (*frequency) + IF;
  }
#ifdef DEBUG
  Serial.println("called AnnounceFrequency()");
#endif
  char freqChars[9], freqToAnnounce[16];
  memset(freqChars,'\0',9);
  memset(freqToAnnounce,'\0',16);
  sprintf(freqChars,"%8i",actualFreq); // right-justify integer frequency [Hz] within freqChars
  // Format tx frequency e.g. "14 060 0" or " 7 115 0", but for rx leave off the MHz, since it is within 1 kHz anyway
  if (bRx) {
    strncpy(freqToAnnounce,freqChars+2,3);
    freqToAnnounce[3] = ' ';
    strncpy(freqToAnnounce+4,freqChars+5,1);
  } else {
    strncpy(freqToAnnounce,freqChars,2);
    freqToAnnounce[2] = ' ';
    strncpy(freqToAnnounce+3,freqChars+2,3);
    freqToAnnounce[6] = ' ';
    strncpy(freqToAnnounce+7,freqChars+5,1);
  }
#ifdef DEBUG
  Serial.println("  will announce: ");
  Serial.println(freqToAnnounce);
#endif
  char stringToSend[128];
  memset(stringToSend,'\0',128);
  for (int iDigit = 0; iDigit < strlen(freqToAnnounce); ++iDigit) {
    char * digit = freqToAnnounce + iDigit;
    switch (*digit) {
      case '0':
        strcat(stringToSend,"-----|");
        break;
      case '1':
        strcat(stringToSend,".----|");
        break;
      case '2':
        strcat(stringToSend,"..---|");
        break;
      case '3':
        strcat(stringToSend,"...--|");
        break;
      case '4':
        strcat(stringToSend,"....-|");
        break;
      case '5':
        strcat(stringToSend,".....|");
        break;
      case '6':
        strcat(stringToSend,"-....|");
        break;
      case '7':
        strcat(stringToSend,"--...|");
        break;
      case '8':
        strcat(stringToSend,"---..|");
        break;
      case '9':
        strcat(stringToSend,"----.|");
        break;
      case '.':
        strcat(stringToSend,".-.-.-|");
        break;
      case ' ':
        strcat(stringToSend," ");
        break;
    }
  }
  strcpy(sidetoneBuffer,stringToSend);
  sidetoneIndex = 0;
  //SidetoneString(stringToSend);
  return;
}

void SidetoneString(const char* morseString) {
  int i, iMax;
  iMax = strlen(morseString);
#ifdef DEBUG
  Serial.println("  translated as: ");
  Serial.println(morseString);
#endif
  for (i = 0; i < iMax; i++) {
    if (morseString[i] == '.') {
      digitalWrite(Side_Tone, HIGH);     // Start tone
      delay(ditTime);                    // Continue tone for the right amount of time
      digitalWrite(Side_Tone, LOW);      // Stop tone
    } else if (morseString[i] == '-') {
      digitalWrite(Side_Tone, HIGH);
      delay(ditTime * CWWeight);
      digitalWrite(Side_Tone, LOW);
    } else if (morseString[i] == '|') { // dah space within word
      delay(ditTime * CWWeight);
    } else { // inter-word space
      delay(ditTime * 7);
    }
  }
  return;
}

// RIT routine  
void RIT_Read()
{
    int RitReadValueNew =0 ;


    RitReadValueNew = analogRead(RitReadPin);
    RitReadValue = (RitReadValueNew + (7 * RitReadValue))/8;//Lowpass filter

    if(RitReadValue < 500) 
        RitFreqOffset = RitReadValue-500;
    else if(RitReadValue < 523) 
        RitFreqOffset = 0;//Deadband in middle of pot
    else 
        RitFreqOffset = RitReadValue - 523;

}

// Check Limits
void  Band_40_Limit_High()
    {
         if ( *frequency < 16.3e6 )
    { 
         stop_led_off();
    } 
    
    else if ( *frequency >= 16.3e6 )
    { 
       *frequency = 16.3e6;
         stop_led_on();    
    }
}
 
void  Band_40_Limit_Low()
    {
        if ( *frequency <= 16.0e6 )  
    { 
        *frequency = 16.0e6;
        stop_led_on();
    } 
    
    else if ( *frequency > 16.0e6 )
    { 
       stop_led_off();
    } 
}
   
void  Band_20_Limit_High()
    {
         if ( *frequency < 5.35e6 )
    { 
         stop_led_off();
    } 
    
    else if ( *frequency >= 5.35e6 )
    { 
       *frequency = 5.35e6;
         stop_led_on();    
    }
}

void  Band_20_Limit_Low()
    {
        if ( *frequency <= 5.0e6 )  
    { 
        *frequency = 5.0e6;
        stop_led_on();
    } 
    
    else if ( *frequency > 5.0e6 )
    { 
        stop_led_off();
    } 
 }

// Frequency set routines
void Default_frequency()
{
    *frequency = frequency_default;
    UpdateFreq(*frequency);

}

// -------------------- Ten*Tec led routines ---------------------
void Step_Flash()
{
    stop_led_on();
    for (int i=0; i <= 25e3; i++); // short delay 
    stop_led_off();   
}

void stop_led_on()
{
    digitalWrite(Band_End_Flash_led, HIGH);
}

void stop_led_off()
{
    digitalWrite(Band_End_Flash_led, LOW);
}

// -------------  Menu routines --------------------------------------

// This routine checks if the right button (labeled FUNCTION) is pressed.
// If so, change the function of the left (SELECT) button.
void Multi_Function()
{
    Step_Multi_Function_Button = digitalRead(Multi_Function_Button);
    if (Step_Multi_Function_Button == HIGH) 
    {  
       // Debounce start
       unsigned long time;
       unsigned long start_time;
      
       time = millis();
       while( digitalRead(Multi_Function_Button) == HIGH ){ 
               
         start_time = time;
         while( (time - start_time) < 7) {
           time = millis();
         }
       }
       // Debounce end

        Step_Multi_Function_Button1++;
        if (Step_Multi_Function_Button1 > 2 ) 
        { 
            Step_Multi_Function_Button1 = 0; 
        }
    }
    // Change function of SELECT button based on Step_Multi_Function_Button1
    Step_Function();
}

void Step_Function()
{
    switch ( Step_Multi_Function_Button1 )
    {
        case 0:
            MF_G(); // change LED
            Step_Select_Button1 = Selected_BW; // Make the hardware setting reflect the software value for that setting.
            Step_Select(); // Changes filter bw, tuning step, or user function
                           // based on the value of Step_Multi_Function_Button1.
            Selection();   // Check to see if the button is pressed and adjust the setting if so.
            break;   //

        case 1:
            MF_Y();
            Step_Select_Button1 = Selected_Step; //
            Step_Select(); //
            Selection();
            break;   //

        case 2: 
            MF_R();
            Step_Select_Button1 = Selected_Other; //
            Step_Select(); //
            Selection();
            break;   //  
    }
}

void  Selection()
{
    Step_Select_Button = digitalRead(Select_Button);
    if (Step_Select_Button == HIGH) 
    {   
       // Debounce start
       unsigned long time;
       unsigned long start_time;
       #ifdef FEATURE_FREQANNOUNCE
         unsigned long long_time;
         long_time = millis();
       #endif
       
       time = millis();
       while( digitalRead(Select_Button) == HIGH ){ 
         
         #ifdef FEATURE_FREQANNOUNCE
           // function button is pressed longer then 2 seconds
           if ( (millis() - long_time) > 2000 && (millis() - long_time) < 2010 ) { 
             // announce frequency
             TX_frequency = ((*frequency) + IF)/100;
             char buffer[8];
             ltoa(TX_frequency, buffer, 10);
             announce(buffer);
        
             // wait for button release
             while( digitalRead(Select_Button) == HIGH ){ 
             }   
             return;        
           } 
         #endif

         start_time = time;
         while( (time - start_time) < 7) {
           time = millis();
         }
       }
       // Debounce end

        Step_Select_Button1 = Step_Select_Button1++;
        if (Step_Select_Button1 > 2 ) 
        { 
            Step_Select_Button1 = 0; 
        }
    }
    // Step_Select() changes filter bw, tuning step, or user function
    // based on the value of Step_Multi_Function_Button1.
    Step_Select(); 
}

void Step_Select()
{
    switch ( Step_Select_Button1 )
    {
        case 0: //   Select_Green   could place the S_G() routine here!
            S_G();
            break;

        case 1: //   Select_Yellow  could place the S_Y() routine here!
            S_Y();
            break; 

        case 2: //   Select_Red    could place the S_R() routine here!
            S_R();
            break;     
    }
}

// The MF_ functions turns on one SELECT lamp, and turns the others off.
void MF_G()    //  Turn on Multi-function Green lamp ONLY.
{
    digitalWrite(Multi_function_Green, HIGH);    
    digitalWrite(Multi_function_Yellow, LOW);  // 
    digitalWrite(Multi_function_Red, LOW);  //
}

void MF_Y()   //  Turn on Multi-function Yellow lamp ONLY.
{
    digitalWrite(Multi_function_Green, LOW);    
    digitalWrite(Multi_function_Yellow, HIGH);  // 
    digitalWrite(Multi_function_Red, LOW);  //
}

void MF_R()   //  Turn on Multi-function Red lamp ONLY.
{
    digitalWrite(Multi_function_Green, LOW);
    digitalWrite(Multi_function_Yellow, LOW);  // 
    digitalWrite(Multi_function_Red, HIGH);
}

// The S_ functions change the filter/tunestep/user fcn depending on the value of Step_Multi_Function_Button1.
void S_G()  // Select Green 
{
    digitalWrite(Select_Green, HIGH); 
    digitalWrite(Select_Yellow, LOW);  // 
    digitalWrite(Select_Red, LOW);  //
    if (Step_Multi_Function_Button1 == 0)  
        Band_Width_W(); 
    else if (Step_Multi_Function_Button1 == 1)  
        Step_Size_100(); 
    else if (Step_Multi_Function_Button1 == 2)  
        Other_1(); 
}

void S_Y()  // Select Yellow
{
    digitalWrite(Select_Green, LOW); 
    digitalWrite(Select_Yellow, HIGH);  // 
    digitalWrite(Select_Red, LOW);  //
    if (Step_Multi_Function_Button1 == 0) 
    {
        Band_Width_M();
    } 
    else if (Step_Multi_Function_Button1 == 1) 
    {
        Step_Size_1k(); 
    }
    else if (Step_Multi_Function_Button1 == 2) 
    {
        Other_2();
    }
}

void S_R()  // Select Red
{
    digitalWrite(Select_Green, LOW);   //
    digitalWrite(Select_Yellow, LOW);  // 
    digitalWrite(Select_Red, HIGH);    //
    if (Step_Multi_Function_Button1 == 0) 
    {
        Band_Width_N();
    } 
    else if (Step_Multi_Function_Button1 == 1) 
    {
        Step_Size_10k(); 
    }
    else if (Step_Multi_Function_Button1 == 2) 
    {
        Other_3(); 
    }
}

void Band_Width_W()
{
    digitalWrite( Medium_A8, LOW);   // Hardware control of I.F. filter shape
    digitalWrite( Narrow_A9, LOW);   // Hardware control of I.F. filter shape
    Selected_BW = Wide_BW; 
}

void Band_Width_M()
{
    digitalWrite( Medium_A8, HIGH);  // Hardware control of I.F. filter shape
    digitalWrite( Narrow_A9, LOW);   // Hardware control of I.F. filter shape
    Selected_BW = Medium_BW;  
}

void Band_Width_N()
{
    digitalWrite( Medium_A8, LOW);   // Hardware control of I.F. filter shape
    digitalWrite( Narrow_A9, HIGH);  // Hardware control of I.F. filter shape
    Selected_BW = Narrow_BW; 
}

void Step_Size_100()                 // Encoder Step Size 
{
    frequency_step = 100;            //  Can change this whatever step size one wants
    Selected_Step = Step_100_Hz; 
}

void Step_Size_1k()                 // Encoder Step Size 
{
    frequency_step = 1e3;           //  Can change this whatever step size one wants
    Selected_Step = Step_1000_hz; 
}

void Step_Size_10k()                // Encoder Step Size 
{
    frequency_step = 10e3;          //  Can change this whatever step size one wants
    Selected_Step = Step_10000_hz; 
}

void Other_1()                      //  User Defined Control Software 
{
    Selected_Other = Other_1_user; 
}

void Other_2()                      //  User Defined Control Software
{
    Selected_Other = Other_2_user; 
}

void Other_3()                      //  User Defined Control Software
{
    Selected_Other = Other_3_user;
}

uint32_t TimerOverFlow(uint32_t currentTime)
{
    return (currentTime + CORE_TICK_RATE*(1));//the Core Tick Rate is 1ms
}

//-----------------------------------------------------------------------------
// ****************  Dont bother the code below  ******************************
// \/  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
//-----------------------------------------------------------------------------
void program_freq0(long frequency)
{
    AD9834_reset_high();  
    int flow,fhigh;
    fcalc = frequency*(268.435456e6 / Reference );    // 2^28 =
    flow = fcalc&0x3fff;              //  49.99975mhz  
    fhigh = (fcalc>>14)&0x3fff;
    digitalWrite(FSYNC_BIT, LOW);  //
    clock_data_to_ad9834(flow|AD9834_FREQ0_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(fhigh|AD9834_FREQ0_REGISTER_SELECT_BIT);
    digitalWrite(FSYNC_BIT, HIGH);
    AD9834_reset_low();
}    // end   program_freq0

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||  
void program_freq1(long frequency)
{
    AD9834_reset_high(); 
    int flow,fhigh;
    fcalc = frequency*(268.435456e6 / Reference );    // 2^28 =
    flow = fcalc&0x3fff;              //  use for 49.99975mhz   
    fhigh = (fcalc>>14)&0x3fff;
    digitalWrite(FSYNC_BIT, LOW);  
    clock_data_to_ad9834(flow|AD9834_FREQ1_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(fhigh|AD9834_FREQ1_REGISTER_SELECT_BIT);
    digitalWrite(FSYNC_BIT, HIGH);  
    AD9834_reset_low();
}  

//------------------------------------------------------------------------------
void clock_data_to_ad9834(unsigned int data_word)
{
    char bcount;
    unsigned int iData;
    iData=data_word;
    digitalWrite(SCLK_BIT, HIGH);  //portb.SCLK_BIT = 1;  
    // make sure clock high - only chnage data when high
    for(bcount=0;bcount<16;bcount++)
    {
        if((iData & 0x8000)) digitalWrite(SDATA_BIT, HIGH);  //portb.SDATA_BIT = 1; 
        // test and set data bits
        else  digitalWrite(SDATA_BIT, LOW);  
        digitalWrite(SCLK_BIT, LOW);  
        digitalWrite(SCLK_BIT, HIGH);     
        // set clock high - only change data when high
        iData = iData<<1; // shift the word 1 bit to the left
    }  // end for
}  // end  clock_data_to_ad9834

//-----------------------------------------------------------------------------
void AD9834_init()      // set up registers
{
    AD9834_reset_high(); 
    digitalWrite(FSYNC_BIT, LOW);
    clock_data_to_ad9834(0x2300);  // Reset goes high to 0 the registers and enable the output to mid scale.
    clock_data_to_ad9834((FREQ0_INIT_VALUE&0x3fff)|AD9834_FREQ0_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(((FREQ0_INIT_VALUE>>14)&0x3fff)|AD9834_FREQ0_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(0x2200); // reset goes low to enable the output.
    AD9834_reset_low();
    digitalWrite(FSYNC_BIT, HIGH);  
}  //  end   init_AD9834()

//----------------------------------------------------------------------------   
void AD9834_reset()
{
    digitalWrite(RESET_BIT, HIGH);  // hardware connection
    for (int i=0; i <= 2048; i++);  // small delay

    digitalWrite(RESET_BIT, LOW);   // hardware connection
}

//-----------------------------------------------------------------------------
void AD9834_reset_low()
{
    digitalWrite(RESET_BIT, LOW);
}

//-----------------------------------------------------------------------------     
void AD9834_reset_high()
{  
    digitalWrite(RESET_BIT, HIGH);
}
//^^^^^^^^^^^^^^^^^^^^^^^^^  DON'T BOTHER CODE ABOVE  ^^^^^^^^^^^^^^^^^^^^^^^^^ 
//=============================================================================

