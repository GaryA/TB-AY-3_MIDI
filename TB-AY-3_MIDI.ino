// TB-AY-3, MIDI_Controlled 8Bit Sound Generater
// Original code by Erik Oostveen
// www.erikoostveen.co.uk
// Oct 14th, 2019
// ... with the help of some code from http://watts.seesaa.net/article/458678673.html

// Refactored by GaryA
// Requires 
// Encoder library: https://github.com/PaulStoffregen/Encoder
// Button library: https://github.com/tigoe/Button
// MIDI library: https://github.com/FortySevenEffects/arduino_midi_library/releases

/* AY-3-8910 Registers */
#define REG_FREQ_A_LO     0
#define REG_FREQ_A_HI     1
#define REG_FREQ_B_LO     2
#define REG_FREQ_B_HI     3
#define REG_FREQ_C_LO     4
#define REG_FREQ_C_HI     5
#define REG_FREQ_NOISE    6
#define REG_IO_MIXER      7
#define REG_LVL_A         8
#define REG_LVL_B         9
#define REG_LVL_C         10
#define REG_FREQ_ENV_L0   11
#define REG_FREQ_ENV_HI   12
#define REG_ENV_SHAPE     13
#define REG_IOA           14
#define REG_IOB           15

/* AY-3-8910 bus modes */
#define INACTIVE  B00
#define READ      B01
#define WRITE     B10
#define ADDRESS   B11

/* include library headers */
#include <MIDI.h>
#include <Encoder.h>
#include <Button.h>

/* define pin mapping */
// Tx pin 0
// MIDI Rx pin 1
// DA2 pin 2
// DA3 pin 3
// DA4 pin 4
// DA5 pin 5
// DA6 pin 6
// DA7 pin 7
// DA0 pin 8
// DA1 pin 9
const int encAPin = 10;
const int freqOutputPin = 11;   // OC2A output pin for ATmega328 boards
const int encBPin = 12;
const int led1Pin = 13;
// BC1 pin A0
// BDIR pin A1
// RST pin A2
const int buttonPin = A3;
const int paramSelPin = A4;
const int led2Pin = A5;
const int soundSelPin = A6;

const int ocr2aVal  = 3; // counter value for 2MHz clock for AY-3-8910

MIDI_CREATE_DEFAULT_INSTANCE();
Encoder myEnc(encAPin, encBPin);
#ifdef PULLUP
Button button = Button(buttonPin, PULLUP);
#else
Button button = Button(buttonPin,INPUT_PULLUP);
#endif

int paramSelValue = 0;
int paramSel = 0;

int soundSelValue = 0;
int soundSel = 0;

long parameterCounter[11] = {0};

typedef struct 
{
  int oldPosition;
  int newPosition;
  int encCount;
  int value;
} param_t;

typedef struct
{
  param_t param[11];
} sound_t;

sound_t sound[5];

const int minVal[11] = {0, 0, 0, 0, 0, 0, 3, 1, 0, 0, 0};
const int maxVal[11] = {15, 255, 15, 255, 15, 255, 31, 255, 6, 16, 16};

const int envelopeTypeList[] = {9,4,8,12,10,14,11};  
// 9 = Just release             [0]
// 4 = Reverse Release          [1]
// 8 = Repeat Release           [2]
// 12 = Reverse Repeat Release  [3]
// 10 = Repeat Release Waves    [4]
// 14 = Reverse Repeat Waves    [5]
// 11 = Release and on          [6]

const int mixerModeList[] = {0,248,199,254,253,251,252,249,246,237,219,238,221,243,247,239,223}; 
//   0 = A+B+C+Noise            [0]
// 248 = A+B+C                  [1]
// 199 = Noise only on A+B+C    [2]
// 254 = A Only                 [3]
// 253 = B only                 [4]
// 251 = C only                 [5]
// 252 = A+B                    [6]
// 249 = B+C                    [7]
// 246 = A+Noise                [8]
// 237 = B+Noise                [9]
// 219 = C+Noise                [10]
// 238 = A+Noise on B           [11]
// 221 = B+Noise on C           [12]
// 243 = C+Noise on A           [13]
// 247 = Noise only on A        [14]
// 239 = Noise only on B        [15]
// 223 = Noise only on C        [16]

void setupSounds(void)
{
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 11; j++)
    {
      sound[i].param[j].oldPosition = 0;
      sound[i].param[j].newPosition = 0;
      sound[i].param[j].encCount = 1;
    }
  }
// --------------------------------------------------------------------------------
// Sound 1 Parameters
// --------------------------------------------------------------------------------
  sound[0].param[0].value = 9;
  sound[0].param[1].value = 10; // Osc A = 2569 = 48Hz
  sound[0].param[2].value = 10;
  sound[0].param[3].value = 12; // Osc B = 3082 = 40Hz
  sound[0].param[4].value = 11;
  sound[0].param[5].value = 14; // Osc C = 3595 = 34Hz
  sound[0].param[6].value = 16; // Noise period = 16 = 7.8kHz
  sound[0].param[7].value = 5;  // Env period = 1280 = 0.15s
  sound[0].param[8].value = 0;  // Env = release only 
  sound[0].param[9].value = 3;  // Mixer = ch A only
  sound[0].param[10].value = 0; // MIDI = all channels
  
// --------------------------------------------------------------------------------
// Sound 2 Parameters
// --------------------------------------------------------------------------------
  sound[1].param[0].value = 64;
  sound[1].param[1].value = 2;  // Osc A = 576 = 217Hz
  sound[1].param[2].value = 66;
  sound[1].param[3].value = 6;  // Osc B = 1602 = 78Hz
  sound[1].param[4].value = 45;
  sound[1].param[5].value = 2;  // Osc C = 557 = 224Hz
  sound[1].param[6].value = 10; // Noise period = 10 = 12.5kHz
  sound[1].param[7].value = 9;  // Env period = 2304 = 0.3s
  sound[1].param[8].value = 0;  // Env = release only
  sound[1].param[9].value = 0;  // Mixer = A + B + C + Noise

// --------------------------------------------------------------------------------
// Sound 3 Parameters
// --------------------------------------------------------------------------------
  sound[2].param[0].value = 64;
  sound[2].param[1].value = 7;  // Osc A = 1856 = 67Hz
  sound[2].param[2].value = 64;
  sound[2].param[3].value = 7;  // Osc B = 1856 = 67Hz
  sound[2].param[4].value = 64;
  sound[2].param[5].value = 7;  // Osc C = 1856 = 67Hz
  sound[2].param[6].value = 3;  // Noise period = 3 = 42kHz
  sound[2].param[7].value = 2;  // Env period = 512 = 65ms
  sound[2].param[8].value = 0;  // Env = release only
  sound[2].param[9].value = 2;  // Mixer = noise only on A, B & C

// --------------------------------------------------------------------------------
// Sound 4 Parameters
// --------------------------------------------------------------------------------
  sound[3].param[0].value = 64;
  sound[3].param[1].value = 7;  // Osc A = 1856 = 67Hz
  sound[3].param[2].value = 64;
  sound[3].param[3].value = 7;  // Osc B = 1856 = 67Hz
  sound[3].param[4].value = 64;
  sound[3].param[5].value = 7;  // Osc C = 1856 = 67Hz
  sound[3].param[6].value = 3;  // Noise period = 3 = 42kHz
  sound[3].param[7].value = 6;  // Env period = 1536 = 0.2s
  sound[3].param[8].value = 0;  // Env = release only
  sound[3].param[9].value = 2;  // Mixer = noise only on A, B & C

// --------------------------------------------------------------------------------
// Sound 5 Parameters
// --------------------------------------------------------------------------------
  sound[4].param[0].value = 100;
  sound[4].param[1].value = 5;  // Osc A = 1380 = 90Hz
  sound[4].param[2].value = 64;
  sound[4].param[3].value = 7;  // Osc B = 1856 = 67Hz
  sound[4].param[4].value = 64;
  sound[4].param[5].value = 7;  // Osc C = 1856 = 67Hz
  sound[4].param[6].value = 5;  // Noise period = 5 = 25kHz
  sound[4].param[7].value = 8;  // Env period = 2048 = 0.26s
  sound[4].param[8].value = 0;  // Env = release only
  sound[4].param[9].value = 11; // Mixer = A + noise on B
// --------------------------------------------------------------------------------
}

// Set up PSG clock
void setupClock()
{
    pinMode(freqOutputPin, OUTPUT); 
    TCCR2A = ((1 << WGM21) | (1 << COM2A0));
    TCCR2B = (1 << CS20);
    TIMSK2 = 0;
    OCR2A = ocr2aVal;
}

// Set up PSG data bus
void setupData(int mode)
{
  switch(mode)
  {
    default:
    case READ:
    case INACTIVE:
      DDRD = B00000000; // Set all D port as input
      DDRB &= ~0x03;    // Set PB1 & PB0 as input too
      break;
    case ADDRESS:
    case WRITE:
      DDRD = B11111111; // Set all D port as output
      DDRB |= 0x03;     // Set PB1 & PB0 as output too
      break;
  }
}

// Set up PSG control bits
void setupControl()
{
  PORTC &= ~B00000011;
  PORTC |=  B00000100;
  DDRC |= B00000111; // added reset signal to PC2 (as output)
}

// Set PSG control value
void setControl(int mode)
{
  PORTC = (PORTC & 111111100) | (mode);
}

// Reset PSG 
void resetPSG()
{
  PORTC = B11111000;
  delay(1); // more than 500us required.
  PORTC = B11111100;
  delayMicroseconds(6);
}

// write to PSG data bus
void setData(unsigned char data)
{
  PORTD = data & 0xFC;
  PORTB = data & 0x03;
} 

// read from PSG data bus
unsigned char getData(void)
{
  return (PIND & 0xFC) | (PINB & 0x03); // bug-fix: PORTx -> PINx to read
}

// write to PSG register
void writeReg(uint8_t reg, uint8_t value)
{
  setupData(ADDRESS);
  setData(reg & 0x0F);
  setControl(ADDRESS);
  delayMicroseconds(3);
  setControl(INACTIVE);
  delayMicroseconds(1);

  setupData(WRITE);
  setData(value);
  delayMicroseconds(1);
  setControl(WRITE);
  delayMicroseconds(5);  
  setControl(INACTIVE);
  setupData(INACTIVE);
}

// read from PSG register
uint8_t readReg(uint8_t reg)
{
  uint8_t ret = 0;
  setupData(ADDRESS);
  setData(reg & 0x0F);
  setControl(ADDRESS);
  delayMicroseconds(3);
  setControl(INACTIVE);
  delayMicroseconds(1);

  setupData(READ);
  delayMicroseconds(1);
  setControl(READ);
  delayMicroseconds(2);
  ret = getData();
  delayMicroseconds(5);  
  setControl(INACTIVE);
  setupData(INACTIVE);
  return ret;
}

void setup()
{
  setupSounds();
  setupClock();
  setupControl();
  setupData(INACTIVE);
  resetPSG(); // added to initialize PSG.
  
  writeReg(REG_LVL_A, 0);
  writeReg(REG_LVL_B, 0);
  writeReg(REG_LVL_C, 0);

  pinMode(led1Pin, OUTPUT); // Blinks when dial is rotated
  dialBlink();
  pinMode(led2Pin, OUTPUT); // Midi Rx LED
  midiBlink();

  MIDI.begin(MIDI_CHANNEL_OMNI); 
//  Serial.begin(9600);           //  TEST :: setup serial
}

void loop()
{
  soundSelValue = analogRead(soundSelPin); 
  // Serial.println(soundSelValue); // TEST
  
  if ( soundSelValue > 840 && soundSelValue < 870 ) { soundSel = 0; }  
  if ( soundSelValue > 670 && soundSelValue < 700 ) { soundSel = 1; } 
  if ( soundSelValue > 500 && soundSelValue < 530 ) { soundSel = 2; }  
  if ( soundSelValue > 330 && soundSelValue < 350 ) { soundSel = 3; } 
  if ( soundSelValue > 150 && soundSelValue < 180 ) { soundSel = 4; } 
  if ( soundSelValue < 10 ) { soundSel = 5; }

  // Serial.println(soundSel); // TEST

  paramSelValue = analogRead(paramSelPin); 
  // Serial.println(paramSelValue); // TEST

  if ( paramSelValue > 930 && paramSelValue < 950 ) { paramSel = 0; }  
  if ( paramSelValue > 860 && paramSelValue < 880 ) { paramSel = 1; } 
  if ( paramSelValue > 790 && paramSelValue < 810 ) { paramSel = 2; }  
  if ( paramSelValue > 720 && paramSelValue < 740 ) { paramSel = 3; } 
  if ( paramSelValue > 650 && paramSelValue < 670 ) { paramSel = 4; } 
  if ( paramSelValue > 580 && paramSelValue < 600 ) { paramSel = 5; } 
  if ( paramSelValue > 500 && paramSelValue < 530 ) { paramSel = 6; } 
  if ( paramSelValue > 420 && paramSelValue < 450 ) { paramSel = 7; } 
  if ( paramSelValue > 340 && paramSelValue < 360 ) { paramSel = 8; } 
  if ( paramSelValue > 240 && paramSelValue < 270 ) { paramSel = 9; } 
  if ( paramSelValue > 130 && paramSelValue < 160 ) { paramSel = 10; } 
  if ( paramSelValue > 5 && paramSelValue <  25 ) { paramSel = 11; }

  // Serial.println(paramSel); // TEST

  // if parameter is MIDI, set sound to 0 as there is only one MIDI
  if (paramSel == 10)
  {
    soundSel = 0;
  }

  // Trigger selected sound when button is pressed
  if( button.uniquePress() )
  {
    midiBlink();
    playSound(soundSel);
  }

  sound[soundSel].param[paramSel].newPosition = myEnc.read();
  
  if ( sound[soundSel].param[paramSel].newPosition != sound[soundSel].param[paramSel].oldPosition )
  {
    sound[soundSel].param[paramSel].encCount++;
    if ( parameterCounter[paramSel] == 0 )
    { 
      sound[soundSel].param[paramSel].encCount = 0;
    }
    if ( (sound[soundSel].param[paramSel].newPosition > sound[soundSel].param[paramSel].oldPosition)
          && (sound[soundSel].param[paramSel].encCount > 1 ))
    { 
      sound[soundSel].param[paramSel].value--; 
      sound[soundSel].param[paramSel].encCount = 0; 
      dialBlink();
    }
    else if ( (sound[soundSel].param[paramSel].newPosition < sound[soundSel].param[paramSel].oldPosition)
              && (sound[soundSel].param[paramSel].encCount > 1 )) 
    { 
      sound[soundSel].param[paramSel].value++; 
      sound[soundSel].param[paramSel].encCount = 0; 
      dialBlink(); 
    }  
    if ( sound[soundSel].param[paramSel].value < minVal[paramSel] )  
    { 
      sound[soundSel].param[paramSel].value = minVal[paramSel];  
    }
    else if ( sound[soundSel].param[paramSel].value > maxVal[paramSel] ) 
    { 
      sound[soundSel].param[paramSel].value = maxVal[paramSel]; 
    }
    sound[soundSel].param[paramSel].oldPosition = sound[soundSel].param[paramSel].newPosition;
    for (int i = 0; i < 11; i++)
    {
      if ( i != paramSel) parameterCounter[i] = 0; 
    }
  }
    
  parameterCounter[paramSel]++; 
  if ( parameterCounter[paramSel] > 1000 ) 
  { 
    parameterCounter[paramSel] = 1; 
  }

  if (MIDI.read()) 
  {
    switch(MIDI.getType())
    {
     case midi::NoteOn:      
      if ( MIDI.getChannel() == sound[0].param[10].value || sound[0].param[10].value == 0  )
      {
        int voice = MIDI.getData1() - 36;
        if (voice < 8)
        {
          midiBlink();
          playSound(voice);
        }           
      }
    }
  }
}

int dialBlink(void)
{
  digitalWrite(led1Pin, HIGH);
  delay(10); 
  digitalWrite(led1Pin, LOW); 
}

int midiBlink(void)
{
  digitalWrite(led2Pin, HIGH);
  delay(5); 
  digitalWrite(led2Pin, LOW); 
}

void playSound(int i)
{
  if(i == 5)
  {
    sound_5();
  }
  else if (i == 6)
  {
    sound_6();
  }
  else if (i == 7)
  {
    sound_7();
  }
  else
  {
    writeReg (REG_FREQ_A_LO, sound[i].param[0].value); // CH_A: Finetune - Value
    writeReg (REG_FREQ_A_HI, sound[i].param[1].value); // CH_A: Coarse - Higher value is lower frequency
    writeReg (REG_FREQ_B_LO, sound[i].param[2].value); // CH_B: Finetune - Value
    writeReg (REG_FREQ_B_HI, sound[i].param[3].value); // CH_B: Coarse - Higher value is lower frequency
    writeReg (REG_FREQ_C_LO, sound[i].param[4].value); // CH_C: Finetune - Value
    writeReg (REG_FREQ_C_HI, sound[i].param[5].value); // CH_C: Coarse - Fixed Higher value is lower frequency

    writeReg (REG_FREQ_NOISE, sound[i].param[6].value); // Reg 6 = Noise Period: 00 = high, FF = low

    writeReg (REG_IO_MIXER, mixerModeList[sound[i].param[9].value]); // Mixer (R7), 0x00 = Sound and noise on all channels, 0xF8 = Sound only on all channels, 0xC7 = Noise only on all channels

    writeReg (REG_LVL_A, 0x10); // Amplitude Control (R10 = Channel A): 0x10 = Variable Level (follow set envelope values), 0x0(0-F) = Fixed Level (Ignore envelope values)
    writeReg (REG_LVL_B, 0x10); // Amplitude Control (R11 = Channel B): 0x10 = Variable Level (follow set envelope values), 0x0(0-F) = Fixed Level (Ignore envelope values)
    writeReg (REG_LVL_C, 0x10); // Amplitude Control (R12 = Channel C): 0x10 = Variable Level (follow set envelope values), 0x0(0-F) = Fixed Level (Ignore envelope values)

    writeReg (REG_FREQ_ENV_HI, sound[i].param[7].value); // Envelope Control Coarse (R14): Sustain -- 0x00 (short) to 0xFF (long)
    writeReg (REG_ENV_SHAPE, envelopeTypeList[sound[i].param[8].value]); // Envelope Register (R15) -- 0x06 to 0x0F (0 x 09 = Sustain only)
  }
}

int sound_5(void)
{
    int FineRandNumberA = random(225);
    int CoarseRandNumberA = random(5);
    int FineRandNumberB = random(225);
    int CoarseRandNumberB = random(6);
    int FineRandNumberC = random(225);
    int CoarseRandNumberC = random(7);
            
    writeReg (REG_FREQ_A_LO, FineRandNumberA);
    writeReg (REG_FREQ_A_HI, CoarseRandNumberA);
    writeReg (REG_FREQ_B_LO, FineRandNumberB);
    writeReg (REG_FREQ_B_HI, CoarseRandNumberB);
    writeReg (REG_FREQ_C_LO, FineRandNumberC);
    writeReg (REG_FREQ_C_HI, CoarseRandNumberC);
    writeReg (REG_IO_MIXER, 248);   // A + B + C     
    writeReg (REG_LVL_A, 0x10);
    writeReg (REG_LVL_B, 0x10);
    writeReg (REG_LVL_C, 0x10);
    writeReg (REG_FREQ_ENV_HI, 3);
    writeReg (REG_ENV_SHAPE, 9);    // Just release
}

int sound_6(void) // Non editable Effect
{
    int i, j, k, v; 
    int t1 = 26, t2 = 1; 
    writeReg (REG_FREQ_NOISE,0); 
    writeReg (REG_IO_MIXER,0xF8);   // A + B + C 
    writeReg (REG_FREQ_A_HI, t2); 
    for (i = 255; i > 0; i = i - 10)
    { 
      v = int (i / 16); 
      j = (int (i / 12)% 2); 
      writeReg (REG_LVL_A, v); 
      writeReg (REG_FREQ_A_LO, t1-j * 30); 
      delay (12); 
    } 
    writeReg (REG_LVL_A,0); 
}

int sound_7(void) // // Non editable Effect
{
  for (int i = 0; i <= 8; i++) 
  {
    int FineRandNumber = random(225);
    int CoarseRandNumber = random(5);
  
    writeReg (REG_FREQ_A_LO, FineRandNumber);
    writeReg (REG_FREQ_A_HI, CoarseRandNumber);
    writeReg (REG_IO_MIXER, 254);     //Oscillator A only   
    writeReg (REG_LVL_A, 0x10);
    writeReg (REG_LVL_B, 0x00);
    writeReg (REG_LVL_C, 0x00);
    writeReg (REG_FREQ_ENV_HI, 9);
    writeReg (REG_ENV_SHAPE, 9);      // Just release
    delay(120);
  }
}
