
/*
 * Audio following light bar
 * tomz1392 : 02092023
 * does: |-----
         |---------
         or
         --------|--------
            -----|-----

          or (to be implemented)
          |----        ----|
          |-------  -------|

 */

#include <FastLED.h>
#define NUM_LEDS 109 // number of leds
#define DATA_PIN 3  // Where the LED DATA wire is 
#define AUDIO_ANALOG_INPUT_PIN  A0 // Where the Mic analog wire is 
#define INSENSITIVITY_MIN  100
#define INSENSITIVITY_MAX  1000 //150 magic ----- this is to make the mic insensitive from SW
#define LED_UP_DLY 1
#define AVG_SAMPLES 1000

const uint16_t standstill = 505; // 505 set gain level(subjected to vary with trimpot turns on mic) x0x0x0x magic number

CRGB leds[NUM_LEDS];

uint32_t ledSectionLength = 0;
volatile uint16_t barLevel_g;

uint16_t insensitivity=INSENSITIVITY_MIN;

typedef enum
{
  ORIGIN_AT_ZERO=0,
  ORIGIN_AT_CENTRE,
  ORIGIN_AT_OPEN_ENDS,
  NONE
}ledBarConf_en;

uint32_t ledSectionColorMap[]=
{
CRGB::AliceBlue,
CRGB::Amethyst,
CRGB::AntiqueWhite,
CRGB::Aqua,
CRGB::Aquamarine,
CRGB::Azure,
CRGB::Beige,
CRGB::Bisque,
CRGB::BlanchedAlmond,
CRGB::Blue,
CRGB::BlueViolet,
CRGB::Brown,
CRGB::BurlyWood,
CRGB::CadetBlue,
CRGB::Chartreuse,
CRGB::Chocolate,
CRGB::Coral,
CRGB::CornflowerBlue,
CRGB::Cornsilk,
CRGB::Crimson,
CRGB::Cyan,
CRGB::DarkBlue,
CRGB::DarkCyan,
CRGB::DarkGoldenrod,
CRGB::DarkGray,
CRGB::DarkGrey,
CRGB::DarkGreen,
CRGB::DarkKhaki,
CRGB::DarkMagenta,
CRGB::DarkOliveGreen,
CRGB::DarkOrange,
CRGB::DarkOrchid,
CRGB::DarkRed,
CRGB::DarkSalmon,
CRGB::DarkSeaGreen,
CRGB::DarkSlateBlue,
CRGB::DarkSlateGray,
CRGB::DarkSlateGrey,
CRGB::DarkTurquoise,
CRGB::DarkViolet,
CRGB::DeepPink,
CRGB::DeepSkyBlue,
CRGB::DimGray,
CRGB::DimGrey,
CRGB::DodgerBlue,
CRGB::FireBrick,
CRGB::FloralWhite,
CRGB::ForestGreen,
CRGB::Fuchsia,
CRGB::Gainsboro,
CRGB::GhostWhite,
CRGB::Gold,
CRGB::Goldenrod,
CRGB::Gray,
CRGB::Grey,
CRGB::Green,
CRGB::GreenYellow,
CRGB::Honeydew,
CRGB::HotPink,
CRGB::IndianRed,
CRGB::Indigo,
CRGB::Ivory,
CRGB::Khaki,
CRGB::Lavender,
CRGB::LavenderBlush,
CRGB::LawnGreen,
CRGB::LemonChiffon,
CRGB::LightBlue,
CRGB::LightCoral,
CRGB::LightCyan,
CRGB::LightGoldenrodYellow,
CRGB::LightGreen,
CRGB::LightGrey,
CRGB::LightPink,
CRGB::LightSalmon,
CRGB::LightSeaGreen,
CRGB::LightSkyBlue,
CRGB::LightSlateGray,
CRGB::LightSlateGrey,
CRGB::LightSteelBlue,
CRGB::LightYellow,
CRGB::Lime,
CRGB::LimeGreen,
CRGB::Linen,
CRGB::Magenta,
CRGB::Maroon,
CRGB::MediumAquamarine,
CRGB::MediumBlue,
CRGB::MediumOrchid,
CRGB::MediumPurple,
CRGB::MediumSeaGreen,
CRGB::MediumSlateBlue,
CRGB::MediumSpringGreen,
CRGB::MediumTurquoise,
CRGB::MediumVioletRed,
CRGB::MidnightBlue,
CRGB::MintCream,
CRGB::MistyRose,
CRGB::Moccasin,
CRGB::NavajoWhite,
CRGB::Navy,
CRGB::OldLace,
CRGB::Olive,
CRGB::OliveDrab,
CRGB::Orange,
CRGB::OrangeRed,
CRGB::Orchid,
CRGB::PaleGoldenrod,
CRGB::PaleGreen,
CRGB::PaleTurquoise,
CRGB::PaleVioletRed,
CRGB::PapayaWhip,
CRGB::PeachPuff,
CRGB::Peru,
CRGB::Pink,
CRGB::Plaid,
CRGB::Plum,
CRGB::PowderBlue,
CRGB::Purple,
CRGB::Red,
CRGB::RosyBrown,
CRGB::RoyalBlue,
CRGB::SaddleBrown,
CRGB::Salmon,
CRGB::SandyBrown,
CRGB::SeaGreen,
CRGB::Seashell,
CRGB::Sienna,
CRGB::Silver,
CRGB::SkyBlue,
CRGB::SlateBlue,
CRGB::SlateGray,
CRGB::SlateGrey,
CRGB::Snow,
CRGB::SpringGreen,
CRGB::SteelBlue,
CRGB::Tan,
CRGB::Teal,
CRGB::Thistle,
CRGB::Tomato,
CRGB::Turquoise,
CRGB::Violet,
CRGB::Wheat,
CRGB::White,
CRGB::WhiteSmoke,
CRGB::Yellow,
CRGB::YellowGreen,
CRGB::FairyLight,
CRGB::FairyLightNCC
};

void setup() {

  Serial.begin(9600);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(10);
  pinMode(AUDIO_ANALOG_INPUT_PIN, INPUT);
  ledSectionLength = NUM_LEDS / 6;
  
}


uint16_t getBarLevel_g()
{
  return barLevel_g;
}

uint16_t getLevel()
{
  uint16_t level = analogRead(AUDIO_ANALOG_INPUT_PIN);

  //uint16_t gain = abs((int16_t)level - (int16_t)standstill); // silence appears as a peak!
  uint16_t gain = level > standstill? level - standstill:standstill; // anything less than standstill is silence and no peaks!!!

  uint16_t retval = map(gain, 0, insensitivity, 0, NUM_LEDS); // insensitivity seems to be too intelligent and adjusts itself every 1000 loop cyc
 
  if (retval >= NUM_LEDS)
  {
    retval = NUM_LEDS - 2;
  }
  return retval;
}

const colorIdxData[6]={9,33,27,51,64,119};
/*
* set color at indexed led
*/
void setLedAt(uint16_t ledIdx)
{
  /*uint16_t colorMapIdx=getBarLevel_g();
  leds[ledIdx]=ledSectionColorMap[colorMapIdx<149?colorMapIdx:148]; */

 if (ledIdx < ledSectionLength)
  {
    //leds[ledIdx] = CRGB::Blue;
    leds[ledIdx]=ledSectionColorMap[colorIdxData[0]];
  }
  else if (ledSectionLength <= ledIdx && ledIdx < (ledSectionLength * 2) )
  {
    //leds[ledIdx] = CRGB::Green;
    leds[ledIdx]=ledSectionColorMap[colorIdxData[1]];
  }
  else if (ledSectionLength*2 <= ledIdx && ledIdx < (ledSectionLength * 3) )
  {
    //leds[ledIdx] = CRGB::Green;
    leds[ledIdx]=ledSectionColorMap[colorIdxData[2]];
  }
  else if (ledSectionLength*3 <= ledIdx && ledIdx < (ledSectionLength * 4) )
  {
    //leds[ledIdx] = CRGB::Green;
    leds[ledIdx]=ledSectionColorMap[colorIdxData[3]];
  }
  else if (ledSectionLength*4 <= ledIdx && ledIdx < (ledSectionLength * 5) )
  {
    //leds[ledIdx] = CRGB::Green;
    leds[ledIdx]=ledSectionColorMap[colorIdxData[4]];
  }
  else
  {
    //leds[ledIdx] = CRGB::Red;
    leds[ledIdx]=ledSectionColorMap[colorIdxData[5]];
  }
  
}



void setLedAtColorMapped(uint16_t ledIdx)
{

  uint8_t colorMapIdx=getBarLevel_g();

  leds[ledIdx]=ledSectionColorMap[colorMapIdx<149?colorMapIdx:148]; 
  
}

/*
*Starts at one end
*/
void setLedBarLvlOriginZero(uint16_t lvl)
{
  
  static uint16_t oldLvl;
  uint16_t ledIdx = 0;
  if (oldLvl < lvl)
  {
    uint16_t ledIdx = oldLvl;
    while (ledIdx <= lvl)
    {
      setLedAt(ledIdx);
      ledIdx++;
     
      //delay(LED_UP_DLY);
    }
    FastLED.show();
  }
  else if (oldLvl > lvl)
  {
    uint16_t lvlDiff = oldLvl - lvl;
    uint16_t ledIdx = oldLvl;
    while (ledIdx > lvl)
    {
      leds[ledIdx] = CRGB::Black;
      ledIdx--;
    
      
    }
    FastLED.show();

  }

  oldLvl = lvl;
}

/*
*Mirrored at center
*/
void setLedBarLvlOriginCentre(uint16_t lvl)
{
  static uint16_t oldLvl;
  const uint16_t midPoint= NUM_LEDS/2;
  volatile uint16_t ledIdx =midPoint;
  volatile uint16_t mirror_offset=ledIdx - oldLvl;
  if (oldLvl < lvl)
  {
    ledIdx += oldLvl;
    
    while (ledIdx <=midPoint+ lvl)
    {
      setLedAt(ledIdx++);
      
      setLedAt(mirror_offset--);
      
     
      //delay(LED_UP_DLY);
    }
    FastLED.show();
  }
  else if (oldLvl > lvl)
  {
    
    ledIdx += oldLvl;
    while (ledIdx > midPoint+lvl)
    {
      leds[ledIdx--] = CRGB::Black;
      
      leds[mirror_offset++] = CRGB::Black;
      
     
      
    }
    FastLED.show();

  }

  oldLvl = lvl;
}




void ledDriveMode(uint8_t mode,uint16_t level)
{
  switch(mode)
  {
    case  ORIGIN_AT_ZERO:
          setLedBarLvlOriginZero(level);
          break;
    case  ORIGIN_AT_CENTRE:
          setLedBarLvlOriginCentre(level/2);        
          break;
    case ORIGIN_AT_OPEN_ENDS:
                          
          break;
     
    default:
          break;
      
  }

}
void simulateRun()
{
  ledDriveMode(ORIGIN_AT_CENTRE,barLevel_g);
  delay(500);
  barLevel_g=barLevel_g>=108?0:barLevel_g+=9;
/*  ledDriveMode(ORIGIN_AT_CENTRE,10);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,20);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,30);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,40);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,50);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,60);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,70);
  delay(500);
  ledDriveMode(ORIGIN_AT_CENTRE,80);
  delay(500);*/

  
}

void adjustSensitivity(uint16_t lvl)
{
  static uint32_t sum=0;
  static uint16_t samp_ctr=0;
  sum+=lvl;
  samp_ctr++;
  if( samp_ctr==AVG_SAMPLES)
  {
    uint16_t avg=sum/AVG_SAMPLES;
    
    if(avg > 95)
    {
      insensitivity=insensitivity<INSENSITIVITY_MAX?insensitivity+INSENSITIVITY_MIN:INSENSITIVITY_MAX;
    }
    else if (avg < 15)
    {
      insensitivity=insensitivity>INSENSITIVITY_MIN?insensitivity-INSENSITIVITY_MIN:INSENSITIVITY_MIN; 
    }
    else
    {
      // SENSITIVITY IS OPTIMUM, LEAVE IT ALONE

    }
    sum=0;
    samp_ctr=0;

  }

}

void loop() {

  barLevel_g = getLevel();
  adjustSensitivity(barLevel_g);
 // Serial.println(barLevel_g);
  ledDriveMode(ORIGIN_AT_CENTRE,barLevel_g);
  
  //simulateRun();
  
  
}
