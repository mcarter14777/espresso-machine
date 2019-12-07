
#include <ADS1115.h>
#include <PID_v1.h>


#define Nextion Serial2

#define boilerPin 15 
#define zcdPin 5
#define flowPin 19
#define pumpPin 18
#define solenoidPin 4
#define echoPin 2
#define triggerPin 0


const int halfCycleUsec = 8333; //half of 60 hz period
const int zcdDelay = 36;  //zero cross detection delay. due to RC rise/fall time
const uint16_t pumpOnDelay[101] =   // delay for power levels mapped to array index 0-100
    {0, 531, 753, 924, 1068, 1196, 1313, 1421, 1521, 1616, 
    1707, 1793, 1877, 1957, 2035, 2110, 2183, 2255, 2324,
    2393, 2460, 2525, 2590, 2654, 2716, 2778, 2839, 2899,
    2958, 3017, 3075, 3133, 3190, 3246, 3303, 3358, 3414,
    3469, 3524, 3578, 3633, 3687, 3740, 3794, 3848, 3901,
    3954, 4007, 4061, 4114, 4167, 4220, 4273, 4326, 4379, 
    4432, 4486, 4539, 4593, 4647, 4701, 4755, 4810, 4864,
    4919, 4975, 5031, 5087, 5144, 5201, 5258, 5316, 5375,
    5435, 5495, 5556, 5617, 5680, 5743, 5808, 5874, 5941,
    6009, 6079, 6150, 6223, 6299, 6376, 6457, 6540, 6626,
    6717, 6812, 6913, 7020, 7137, 7265, 7410, 7581, 7802,
    8333};
const int pumpOffDelay = 4167; //quarter of 60 hz period
unsigned int  level =  0; // variable for pump level

volatile unsigned long zcdLastFall = 0; //variable to record last occurance of zerocross rising edge 
volatile unsigned long zcdLastRise = 0; //variable to record last occurance of zerocross falling edge
volatile unsigned long currentMicros = 0; //variable to record current time in microsec
volatile unsigned long zcdLastMicros = 0;
unsigned long lastPressureChange = 0;
volatile bool zcdNewFall = false; 
volatile bool zcdNewRise = false;

const float millilitersPerPulse = 0.519481;
volatile unsigned long lastFlowPulseMicros = 0;
volatile unsigned long flowPulseCount;
volatile unsigned long flowPulseInterval;
float rateOfFlow;
double flowSetpoint, flowInput, flowOutput;
// flowKp = 6.6, flowKi = 6.25 , flowKd = 0.05;
double flowKp = 2.5, flowKi = 20 , flowKd = 0.05;

float Vout;
float Vtc;
const int numTempMeasurements = 8;
int tempMeasurements[numTempMeasurements];      // the readings from the analog input
int tempMeasurementIndex = 0;              // the index of the current reading
int tempTotal = 0;                  // the running total
float tempAverage = 0; // the average
const int TempInterval = 5;
uint32_t currentTempMillis;
uint32_t previousTempMillis = 0;
double tempSetpoint, tempInput, tempOutput;
double tempKp = 4.5, tempKi =0.125, tempKd = 0.2;
double boilerPWMOutput;
uint32_t windowStartTime;
const int WindowSize = 2000;
bool boiler;

const int numPressureMeasurements = 8;
float pressureMeasurements[numPressureMeasurements];      // the readings from the analog input
int pressureMeasurementIndex = 0;              // the index of the current reading
float pressureTotal = 0;                  // the running total
float pressureAverage = 0;                // the average
float pressureInput, pressureInputBar;
const int pressureInterval = 5;
uint32_t currentPressureMillis;
uint32_t previousPressureMillis = 0;

const double Vref = 1.2362;
int adcval;
bool readingPressure = false;
bool readingTemp = false;

bool brewing=false;
bool brewStopped = true;
bool preinfusion;
bool extracting = false;
float preinfusionFlow;
float preinfusionExitPressure;
unsigned long brewStartTime;
unsigned long extractionStartTime;
unsigned long preinfusionTime;
unsigned long extractionStartPulses;
float millilitersBrewed;
int volumeTarget;
unsigned long extractionTimeTarget;
PID flowPID(&flowInput, &flowOutput, &flowSetpoint, flowKp, flowKi, flowKd, P_ON_M, DIRECT);
PID tempPID(&tempInput, &tempOutput, &tempSetpoint, tempKp, tempKi, tempKd, P_ON_M, DIRECT);
ADS1115 adc;

unsigned long lastActivity = 0;

long mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Nextion_serial_listen() 
{
    if(Nextion.available() > 2)// Read if more then 2 bytes come (we always send more than 2 <#> <len> <cmd> <id>
    {                
        char start_char = Nextion.read();      // Create a local variable (start_char) read and store the first byte on it  
        if(start_char == '#')// And when we find the character #
        {              
          uint8_t len = Nextion.read();      // Create local variable (len) / read and store the value of the second byte
                                            // <len> is the lenght (number of bytes following) 
          unsigned long tmr_1 = millis();
          boolean cmd_found = true;
            
          while(Nextion.available() < len)// Waiting for all the bytes that we declare with <len> to arrive
          {            
            if((millis() - tmr_1) > 100)// Waiting... But not forever...... 
            {    
              cmd_found = false;              // tmr_1 a timer to avoid the stack in the while loop if there is not any bytes on Serial
              break;                            
            }                                     
            delay(1);                            // Delay for nothing delete it if you want
          }                                   
                                               
            if(cmd_found == true)
            {            // So..., A command is found (bytes in Serial buffer egual more than len)
              uint8_t cmd = Nextion.read();  // Create local variable (cmd). Read and store the next byte. This is the command group
                                             
              switch (cmd)
              {                      
                case 'P': /*or <case 0x50:>  IF 'P' matches, we have the command group "Page". */
//                  first_refresh_page((uint8_t)Nextion.read());  
                break;
                case 'E': //event
                    receive_event((uint8_t)Nextion.read());                          // printh 23 02 45 xx               
                break;  
                case 'B': //brewSettings     //printh 23 09 42
                    receive_brew_settings((uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read());                          // printh 23 02 45 xx               
                break;
                case 'H': //heaterSettings     //printh 23 05 48
                                             //auto off(bool)          auto off time           brew temp                 steam temp
                    receive_heater_settings((uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read(),(uint8_t)Nextion.read());
                break;                                                
              }
            }
        } 
    }    
}

void receive_number(byte nextion_attr, byte val)
{
  if(nextion_attr == 0x01) tempSetpoint = val;
  
}

void receive_event(byte event)
{
  if(event == 0x00) boilerOn();
  if(event == 0x01) boilerOff();
  if(event == 0x02) brewStart();
//  if(event == 0x03) brewStop();
  if(event == 0x04) steamStart();
}
bool donePreinfusing=false;
bool intelliFlow = true;
bool flowProfiling = false;
int flowProfilingRate;
int fixedPressure; 
int brewTemp;
int steamTemp;
bool autoOff = true;
unsigned long autoTime;

void receive_heater_settings(byte sAutoOff, byte sAutoTime, byte sBrewTemp, byte sSteamTemp)
{
  autoOff = (bool)sAutoOff;
  Serial.print("autooff");
  Serial.println(autoOff);
  autoTime = (int)sAutoTime * 1000 * 60;
  Serial.print("autotime");
  Serial.println(autoTime);
  brewTemp = (int)sBrewTemp;
  Serial.print("brewtemp");
  Serial.println(brewTemp);  
  steamTemp = (int)sSteamTemp;
  Serial.print("steamtemp");
  Serial.println(steamTemp);    
}

void receive_brew_settings(byte sPreinfusion, byte sPreinfusionTime, byte sPreinfusionFlow, byte sPreinfusionExitPressure, byte sExtractionMode, byte sVolumeTarget, byte sExtractionTimeTarget, byte sflowProfilingRate)
{
  preinfusion = (bool)sPreinfusion;
  Serial.println(preinfusion);
  preinfusionTime = (long)sPreinfusionTime * 1000;
  Serial.println(preinfusionTime);
  preinfusionFlow = (float)sPreinfusionFlow/10;
  Serial.println(preinfusionFlow);
  preinfusionExitPressure = (float)sPreinfusionExitPressure/10;
  Serial.println(preinfusionExitPressure);
  switch(sExtractionMode)
  {
    case '0': //intelliflow
      intelliFlow = true;
      flowProfiling = false;
    break;
    case '1': //flowprofiling
      intelliFlow = false;
      flowProfiling = true;
    break;
    case '2': //
      intelliFlow = false;
      flowProfiling = false;
    break;
  }
  Serial.println(intelliFlow);
  Serial.println(flowProfiling);
  volumeTarget = (int)sVolumeTarget;
  Serial.println(volumeTarget);
  extractionTimeTarget = (int)sExtractionTimeTarget*1000;
  Serial.println(extractionTimeTarget);
  flowProfilingRate = (int)sflowProfilingRate;
  Serial.println(flowProfilingRate);
}

void boilerOn()
{
  tempPID.SetMode(AUTOMATIC);
  boiler=true;
  Serial.println("boileron");
}

void boilerOff()
{
  tempPID.SetMode(MANUAL);
  digitalWrite(boilerPin, LOW);
  tempOutput=0;
  Serial.println("boileroff");
//  Nextion.print("homevboiler.val=0");
//  Nextion.print("\xFF\xFF\xFF");
  boiler=false;
}

void brewStart()
{
  brewStartTime = millis();
  //Serial.println(brewStartTime);
  brewing = true;
  brewStopped = false;
  extracting = false;
  donePreinfusing = false;
  //preinfusion = true;
  digitalWrite(solenoidPin, HIGH);
}

void steamStart()
{
  
}


/////////
void brewControl()
{
  if(brewing)
  {
    if(preinfusion==1&&!donePreinfusing)
    {
      preinfusionControl();
    }
    else
    {
      if(!extracting)
      {
        extractionStartPulses = flowPulseCount;
        flowPID.SetMode(AUTOMATIC);
        extracting=true;
      }
      extractionControl();
    }
  }
  else if (!brewStopped)
  {
    flowPID.SetMode(MANUAL);
    level = 0;
    flowSetpoint = 0;
    phaseControl();
    digitalWrite(solenoidPin, LOW);
    delay(1000);
    digitalWrite(solenoidPin, HIGH);
    delay(100);
    digitalWrite(solenoidPin, LOW);
    delay(100);
    digitalWrite(solenoidPin, HIGH);
    delay(100);
    digitalWrite(solenoidPin, LOW);  
    delay(100);
    digitalWrite(solenoidPin, HIGH);
    delay(100);
    digitalWrite(solenoidPin, LOW);  
    delay(100);
    digitalWrite(solenoidPin, HIGH);
    delay(100);
    digitalWrite(solenoidPin, LOW); 
    delay(100);
    digitalWrite(solenoidPin, HIGH);
    delay(100);
    digitalWrite(solenoidPin, LOW);     
    delay(100);
    digitalWrite(solenoidPin, HIGH);
    delay(100);
    digitalWrite(solenoidPin, LOW);         
    brewStopped = true;
  }
}
//int preinfusionExitPressure = 58;
int preinfusionLevel = 40;

void preinfusionControl()
{
  if((millis() - brewStartTime >= preinfusionTime) || (pressureInputBar >= preinfusionExitPressure))
  {
    donePreinfusing = true;
    //preinfusionTime = millis() - brewStartTime;
    //extractionStartPulses = flowPulseCount;
    //flowPID.SetMode(AUTOMATIC);
  }
  else
  {
    flowPID.SetMode(AUTOMATIC);
    flowSetpoint = preinfusionFlow;
    flowControl();
  }
}

void extractionControl()
{
  if(intelliFlow)
  {
    if(millis() - brewStartTime >= extractionTimeTarget)
    {
      brewing = false;
    }
    else
    {
    millilitersBrewed = (flowPulseCount - extractionStartPulses)*millilitersPerPulse;
    flowSetpoint = (volumeTarget - millilitersBrewed) / ((float)(extractionTimeTarget - (millis() - brewStartTime))/1000);
    flowSetpoint = constrain(flowSetpoint, 0.0, 10.0);    
    flowControl();
    }
  }
  else if(flowProfiling)
  {
    if(millis() - brewStartTime >= extractionTimeTarget)
    {
      brewing = false;
    }
    else
    {
    flowSetpoint = flowProfilingRate;   
    flowControl();
    }
  }
  else
  {
    if(millis() - brewStartTime >= extractionTimeTarget)
    {
      brewing = false;
    }
    else
    {   
      level = fixedPressure;
    }
  }
}


void IRAM_ATTR zcdFall()
{
 
  zcdLastFall = micros(); //sets equal to current microseconds
  zcdNewFall = true;
 
}

void IRAM_ATTR zcdRise()
{

  zcdLastRise = micros(); //sets equal to current microseconds
  zcdNewRise = true;

}

void IRAM_ATTR zcdISR()
{
  if( micros()- zcdLastMicros >= 3000)
  {

    if( digitalRead(zcdPin) == HIGH) zcdRise();
    else zcdFall();    
    zcdLastMicros = micros();

  }
}

void phaseControl()
{

  //currentMicros = micros();  //records current time in microseconds
  if(zcdNewFall)
  {
    if(micros() - zcdLastFall >= 4167) 
    {
      
      digitalWrite(pumpPin, LOW);  //turns pump off
      zcdNewFall = false;

    } 
  }

  //currentMicros = micros();
  if(zcdNewRise)
  {
    if(micros() - zcdLastRise >= 8333 - pumpOnDelay[level])
    {
      digitalWrite(pumpPin, HIGH);//turns pump on
      zcdNewRise = false; 

    }  
  }
  if(level==0)
  {
    digitalWrite(pumpPin, LOW);
  }
}

void IRAM_ATTR flowISR()
{
  if( micros() - lastFlowPulseMicros >= 100)
  {
    if(digitalRead(flowPin) == LOW) 
    {
      flowPulseCount++;
      flowPulseInterval = micros() - lastFlowPulseMicros;
      lastFlowPulseMicros = micros();
    }
  }
}

float readFlowRate() // returns milliliters per second flow rate
{
  if(micros() - lastFlowPulseMicros > 2000000)
  {
    return 0;
  }
  else if(flowPulseInterval)
  {
    return ( millilitersPerPulse / ((float)flowPulseInterval/1000000));
  }
  else return 0;
}

void flowControl()
{
  rateOfFlow= readFlowRate();
  flowInput = rateOfFlow;
  flowPID.Compute();
  level = (int)flowOutput;
}

int readADC()
{
  static int read_triggered = 0;
  if (!read_triggered) 
  {
    if (adc.trigger_sample() == 0)
      read_triggered = 1;
  } 
  else 
  {
    if (!adc.is_sample_in_progress()) 
    {
      adcval = adc.read_sample();
      read_triggered = 0;
    }
  }
  return adcval;
}


void readPressure()
{
  currentPressureMillis = millis();
  if (currentPressureMillis - previousPressureMillis > 5) 
  {
    // subtract the last reading:
    pressureTotal = pressureTotal - pressureMeasurements[pressureMeasurementIndex];
    pressureMeasurements[pressureMeasurementIndex] = analogRead(32);//pin VP
    //Serial.println("readingpressure");
    pressureTotal = pressureTotal + pressureMeasurements[pressureMeasurementIndex];
    pressureMeasurementIndex = pressureMeasurementIndex + 1;
    if (pressureMeasurementIndex >= numPressureMeasurements)
    {
      pressureMeasurementIndex = 0;
    }
    pressureAverage = pressureTotal / numPressureMeasurements;
    float pressureVoltage = (pressureAverage * 3.3) / 4095;
    pressureVoltage = constrain(pressureVoltage, 0.3348, 3.3);
    pressureInput = (pressureVoltage * 113) - 37.8;
    if(pressureInput < 0) pressureInput = 0;
    pressureInputBar = pressureInput*0.06895;
    previousPressureMillis = currentPressureMillis;
  }
  //return pressureInput*0.06895;
}



double readTemp()
{
  currentTempMillis = millis();
  if (currentTempMillis - previousTempMillis > 20) 
  {
    // subtract the last reading:
    tempTotal = tempTotal - tempMeasurements[tempMeasurementIndex];
    tempMeasurements[tempMeasurementIndex] = readADC();
    //Serial.println("readingTemp");
    tempTotal = tempTotal + tempMeasurements[tempMeasurementIndex];
    tempMeasurementIndex = tempMeasurementIndex + 1;
    if (tempMeasurementIndex >= numTempMeasurements)
    {
      tempMeasurementIndex = 0;
    }

    tempAverage = tempTotal / numTempMeasurements;
    Vout = tempAverage * 0.0625 / 1000;
    tempInput =  (Vout - 1.25) / 0.005;
    previousTempMillis = currentTempMillis;
  }
  return tempInput;
}

void boilerControl(void)
{
  if(boiler)
  {
  readTemp();
  tempPID.Compute();
  

  boilerPWMOutput = tempOutput * (WindowSize / 100.00);
  // Starts a new PWM cycle every WindowSize milliseconds
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  // Calculate the number of milliseconds that have passed in the current PWM cycle.
  // If that is less than the Output value, the relay is turned ON
  // If that is greater than (or equal to) the Output value, the relay is turned OFF.
  if (boilerPWMOutput > (millis() - windowStartTime))
  {
    digitalWrite(boilerPin, HIGH);
  } else {
    digitalWrite(boilerPin, LOW);
  }
  }
}


void setupADC()
{
  adc.begin();
  adc.set_data_rate(ADS1115_DATA_RATE_860_SPS);
  adc.set_mode(ADS1115_MODE_SINGLE_SHOT);
  adc.set_mux(ADS1115_MUX_GND_AIN0); //to be set when reading value
  adc.set_pga(ADS1115_PGA_TWO);
}
char temp_data[50];
unsigned long lastNextionPrint1000=0;
unsigned long lastNextionPrint500=0;
unsigned long lastGraphDataSend=0;
unsigned long lastWaterLevelTrigger=0;
unsigned long lastWaterLevelRead=0;
float graphPressure;
float graphFlow;
float graphFlowSetpoint;
float cm;
bool led2State;
void coreTask( void * pvParameters )
{
    delay(400);
    while(true)
    {
      if(millis()- lastNextionPrint1000 >= 1000)
      { 
        Serial.println("1000");
        sprintf(temp_data, "home.t0.txt=\"%dc\"", (int)tempInput);
        Nextion.print(temp_data);
        Nextion.print("\xFF\xFF\xFF");
        if(boiler)
        {
          Nextion.print("home.vboiler.val=1");
          Nextion.print("\xFF\xFF\xFF");
        }
        else
        {
          Nextion.print("home.vboiler.val=0");
          Nextion.print("\xFF\xFF\xFF");          
        }
        Serial.println(pressureInputBar);
        Serial.println(graphPressure);
        Serial.println(tempInput);  
        Serial.println(cm); 
        lastNextionPrint1000 = millis();            
      } 
      if((millis()- lastNextionPrint500 >= 500)&&!brewing)
      { 
        
        if(boiler&&(tempInput >= (tempSetpoint - 1)))
        {
          if(led2State)
          {
          Nextion.print("pio1=0");
          Nextion.print("\xFF\xFF\xFF");
          led2State=false;
          }
          else
          {
          Nextion.print("pio1=1");
          Nextion.print("\xFF\xFF\xFF");
          led2State=true;
          }
          Serial.println("test");
        }
        else
        {
          Nextion.print("pio1=0");
          Nextion.print("\xFF\xFF\xFF");          
        }   
        lastNextionPrint500 = millis();           
      }

      if(brewing)
      {
        if(millis()-lastGraphDataSend >= 108)
        {
          graphPressure = mapf(pressureInputBar, 0, 15, 0, 200);
          graphFlow = mapf(readFlowRate(), 0, 10, 0, 200); //max 9.7
          graphFlowSetpoint = mapf(flowSetpoint, 0, 10, 0, 200);
          sprintf(temp_data, "add 1,0,%d", (int)graphPressure);
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          sprintf(temp_data, "add 1,1,%d", (int)graphFlow);
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          sprintf(temp_data, "add 1,2,%d", (int)graphFlowSetpoint);
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          sprintf(temp_data, "brew.x0.val=%d", (int)(pressureInputBar*10));
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          sprintf(temp_data, "brew.x1.val=%d", (int)(readFlowRate()*10));
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          sprintf(temp_data, "brew.n0.val=%d", (int)tempInput);
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          sprintf(temp_data, "brew.x3.val=%d", (int)(flowSetpoint*10));
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          Nextion.print("pio1=1");
          Nextion.print("\xFF\xFF\xFF");
          if(boiler)
          {
          sprintf(temp_data, "brew.n1.val=%d", (int)(tempSetpoint));
          Nextion.print(temp_data);
          Nextion.print("\xFF\xFF\xFF");
          }
          lastGraphDataSend = millis();          
        }
      }

      //delay(1000);
      Nextion_serial_listen();
      //  Serial.print("level");
       // Serial.println(level);
    }
 
}
 
void setup()
{
  Nextion.begin(250000, SERIAL_8N1, 23, 26);
  Nextion.setRxBufferSize(1024);
  Serial.begin(115200);
  delay(1000);
  pinMode(echoPin, INPUT_PULLDOWN);
  pinMode(triggerPin, OUTPUT);
  pinMode(zcdPin, INPUT_PULLUP); //zcd  (zero cross detector) pin
  pinMode(flowPin, INPUT_PULLUP); //flow sensor pin
  pinMode(pumpPin, OUTPUT); //pump pin 
  pinMode(15, OUTPUT); //boiler pin
  pinMode(solenoidPin, OUTPUT); //Solenoid pin 
  digitalWrite(solenoidPin, LOW); //turn on to allow water to flow out of goup head
  //digitalWrite(solenoidPin, HIGH);
  digitalWrite(boilerPin,LOW);

  attachInterrupt(digitalPinToInterrupt(flowPin), flowISR, FALLING); //attaches interrupt to flow pin and calls flowISR() on rising or falling edge
  attachInterrupt(digitalPinToInterrupt(zcdPin), zcdISR, CHANGE); //attaches interrupt to zcd pin and calls zcdISR() on rising or falling edge
  
  setupADC();
  tempPID.SetOutputLimits(0, 100);
  flowPID.SetOutputLimits(30,100);
  flowPID.SetMode(MANUAL);
  tempPID.SetMode(MANUAL);
  flowSetpoint = 0;
  tempSetpoint = 93;
  xTaskCreatePinnedToCore(
                    coreTask,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0);  /* Core where the task should run */
 
  Serial.println("Task created...");
  boiler=false;

}

void loop()
{
  //currentMicros = micros();
  //level = 100;
  readTemp();
  boilerControl();
  readFlowRate();
  readPressure();
  //flowControl();
  brewControl();
  phaseControl();
  //delayMicroseconds(100);
}
