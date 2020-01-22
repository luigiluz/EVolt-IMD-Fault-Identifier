/*
 * IMD fault identifier
 * Autor: Luigi Luz
 * Date: 21/01/2020
 */

/* IMD conditions */
#define ERR_FREQ_INVALID           -2
#define ERR_DC_INVALID            -1
#define SUCCESSFULL_INS_MEASUREMENT_NC    0
#define SUCCESSFULL_INS_MEASUREMENT_UVC   1
#define SPEED_START_MEASUREMENT_GOOD    2
#define SPEED_START_MEASUREMENT_BAD     3
#define DEVICE_ERROR_DETECTED       4
#define FAULT_DETECTED_ON_EARTH_CONNECTION  5

/* Time interval to get signal frequency */
#define TIME_INTERVAL           1000
/* Confiability range */
#define FRQ_DEVIATION           2
#define DC_DEVIATION            2

/* IMD Frequency Limits */
#define NORMAL_FREQ             10
#define NORMAL_FREQ_MIN           NORMAL_FREQ - FRQ_DEVIATION
#define NORMAL_FREQ_MAX           NORMAL_FREQ + FRQ_DEVIATION
#define NORMAL_DC_MIN           5 - DC_DEVIATION
#define NORMAL_DC_MAX           95 + DC_DEVIATION
#define UNDERVOLTAGE_FREQ         20
#define UNDERVOLTAGE_FREQ_MIN       UNDERVOLTAGE_FREQ - FRQ_DEVIATION
#define UNDERVOLTAGE_FREQ_MAX         UNDERVOLTAGE_FREQ + FRQ_DEVIATION
#define UNDERVOLTAGE_DC_MIN         NORMAL_DC_MIN
#define UNDERVOLTAGE_DC_MAX         NORMAL_DC_MAX
#define SPEED_START_FREQ          30
#define SPEED_START_FREQ_MIN        SPEED_START_FREQ - FRQ_DEVIATION
#define SPEED_START_FREQ_MAX        SPEED_START_FREQ + FRQ_DEVIATION
#define SPEED_START_DC_GOOD_MIN       5 - DC_DEVIATION
#define SPEED_START_DC_GOOD_MAX       10 + DC_DEVIATION
#define SPEED_START_DC_BAD_MIN        90 - DC_DEVIATION
#define SPEED_START_DC_BAD_MAX        90 + DC_DEVIATION
#define DEVICE_ERROR_FREQ           40
#define DEVICE_ERROR_FREQ_MIN       DEVICE_ERROR_FREQ - FRQ_DEVIATION
#define DEVICE_ERROR_FREQ_MAX       DEVICE_ERROR_FREQ + FRQ_DEVIATION
#define DEVICE_ERROR_DC_MIN         47.5 - DC_DEVIATION 
#define DEVICE_ERROR_DC_MAX         52.5 + DC_DEVIATION
#define CONNECTION_FAULT_FREQ         50
#define CONNECTION_FAULT_FREQ_MIN     CONNECTION_FAULT_FREQ - FRQ_DEVIATION
#define CONNECTION_FAULT_FREQ_MAX       CONNECTION_FAULT_FREQ + FRQ_DEVIATION
#define CONNECTION_FAULT_DC_MIN       DEVICE_ERROR_DC_MIN
#define CONNECTION_FAULT_DC_MAX       DEVICE_ERROR_DC_MAX

/* -------------------------------------------------------------- */

int IMDcondition(int frequency, int dc){
  if (frequency > NORMAL_FREQ_MIN && frequency < NORMAL_FREQ_MAX){
    if (dc > NORMAL_DC_MIN && dc < NORMAL_DC_MAX){
      
      return SUCCESSFULL_INS_MEASUREMENT_NC;
    }
    else {
      
      return ERR_DC_INVALID;
    }
    
  }
  else if (frequency > UNDERVOLTAGE_FREQ_MIN && frequency < UNDERVOLTAGE_FREQ_MAX){
    if (dc > UNDERVOLTAGE_DC_MIN && dc < UNDERVOLTAGE_DC_MAX){
      
      return SUCCESSFULL_INS_MEASUREMENT_UVC;
    }
    else {
      
      return ERR_DC_INVALID;  
    }
  }
  else if (frequency > SPEED_START_FREQ_MIN && frequency < SPEED_START_FREQ_MAX){
    if (dc > SPEED_START_DC_GOOD_MIN && dc < SPEED_START_DC_GOOD_MAX) {
      
      return SPEED_START_MEASUREMENT_GOOD;
    }
    else if (dc > SPEED_START_DC_BAD_MIN && dc < SPEED_START_DC_BAD_MAX) {
    
      return SPEED_START_MEASUREMENT_BAD;
    }
    else {
    
      return ERR_DC_INVALID;
    }
  }
  else if (frequency > DEVICE_ERROR_FREQ_MIN && frequency < DEVICE_ERROR_FREQ_MAX){
    if (dc > DEVICE_ERROR_DC_MIN && dc < DEVICE_ERROR_DC_MAX) {
      
      return DEVICE_ERROR_DETECTED;
    }
    else {
    
      return ERR_DC_INVALID;
    }
  }
  else if (frequency > CONNECTION_FAULT_FREQ_MIN && frequency < CONNECTION_FAULT_FREQ_MAX){
    if (dc > CONNECTION_FAULT_DC_MIN && dc < CONNECTION_FAULT_DC_MAX) {
    
      return FAULT_DETECTED_ON_EARTH_CONNECTION;
    }
    else {
    
      return ERR_DC_INVALID;
    }
  }
  else {
    
    return ERR_FREQ_INVALID;
  }
}

/* -------------------------------------------------------------- */

/* Variable to count transition events */
volatile int edgeCount = 0;

unsigned int frequency;
unsigned long duty;
unsigned long lastToggleTime = 0;

int dutyCycle(int INPUT_PIN){
  unsigned long onTime, offTime, period, dutyCycle;
    onTime = pulseIn(INPUT_PIN,HIGH);
    offTime = pulseIn(INPUT_PIN,LOW);
    period = onTime + offTime;
    dutyCycle = (onTime*100)/period; 
  
    return dutyCycle;
}

void configEI(){
  /*
   * ISC0(1:0) = 11
   * The falling edge on INT0 generates an interupt request
   */
  EICRA |= (1<<ISC01);
  EIMSK |= (1<<INT0);
}

void configPin(){
  DDRD &= ~(1<<DDD2);
}

/* Interrupt service routine for external interruptions */
ISR(INT0_vect){
  EIMSK &= ~(1<<INT0);  
    delayMicroseconds(100);
    edgeCount++;
    EIMSK |= (1<<INT0);
}

void setup()
{
  Serial.begin(9600);
  configEI();
  configPin();
}

void loop()
{
  if(millis() - lastToggleTime >= TIME_INTERVAL){
    duty = dutyCycle(2);
    frequency = edgeCount;
    
    int IMDFault = IMDcondition(frequency, duty);
    
    Serial.print("IMD condition code: ");
    Serial.println(IMDFault);
    
    Serial.print("IMD condition: ");
    if (IMDFault == SUCCESSFULL_INS_MEASUREMENT_NC){
      Serial.print("Normal condition");
  }
    else if (IMDFault == SUCCESSFULL_INS_MEASUREMENT_UVC){
      Serial.print("Undervoltage condition");
    }
    else if (IMDFault == SPEED_START_MEASUREMENT_GOOD){
      Serial.print("Speed start measurement - Good");
    }
    else if (IMDFault == SPEED_START_MEASUREMENT_BAD){
      Serial.print("Speed start measurement - Bad");
    }
    else if (IMDFault == DEVICE_ERROR_DETECTED){
      Serial.print("Device error");
    }
    else if (IMDFault == FAULT_DETECTED_ON_EARTH_CONNECTION){
      Serial.print("Connection fault earth");
    }
    else {
      Serial.print("Invalid frequency");
    }
    Serial.print(" | Frequency: ");
    Serial.print(frequency);
    Serial.print(" Hz | ");
    Serial.print("Duty Cycle: ");
    Serial.print(duty);
    Serial.println("%");
    
    edgeCount = 0;
    lastToggleTime = millis();
  }
}
