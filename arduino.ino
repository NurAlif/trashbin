#include <Arduino.h>
#include <SoftwareSerial.h>
#define DEVICE_ADDR 10//this device address
#define PIN_TXRX 4
#define USE_INTERNAL_PULLUP true //enable internal pullup resistor

SoftwareSerial sb = SoftwareSerial(2,3);

#define ID1   1
#define ID2   2
	


#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36


unsigned long lastTime = 0;

int targetA = 0;
int targetB = 0;

int posTarA[4] = {710, 1084, -16, 317};
int posIA = 0;
int interval =  2000;
int tolerance = 50;
int toleranceB = 18;
int currentPosIA  = 0;

bool triggerOpen = false;
int openTimeWindow = 500;
int stateLid = 0;
unsigned long startWindow = 0;

#define STATELID_closed   0
#define STATELID_opening  1
#define STATELID_window   2
#define STATELID_closing  3

//#define LOBOT_DEBUG 1  /*Debug ：print debug value*/

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(SoftwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}

void LobotSerialServoStopMove(SoftwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

void LobotSerialServoSetID(SoftwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoSetMode(SoftwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  SerialX.write(buf, 10);
}
void LobotSerialServoLoad(SoftwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoUnload(SoftwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}


int LobotSerialServoReceiveHandle(SoftwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}


int LobotSerialServoReadPosition(SoftwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
int LobotSerialServoReadVin(SoftwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int posA = 0;
int posB = 0;

bool lidOpen = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

	Serial.begin(9600);	
  sb.begin(115200);
  delay(2000);
  Serial.println("Started");
  LobotSerialServoSetMode(sb, ID1, 1, 0);
  LobotSerialServoSetMode(sb, ID2, 1, 0);
  posA = LobotSerialServoReadPosition(sb, ID1);
  posB = LobotSerialServoReadPosition(sb, ID2);
  if(posB > 1030 && posB < 1180){ // buka
    lidOpen = true;
    Serial.println("pos lid open");
  }
  else if(posB > 500 && posB < 760){
    lidOpen = false;
    Serial.println("pos lid closed");

  }else{
    while (true)
    {
      delay(1000);
      Serial.print("pos lid not correct : ");
      Serial.print(posB);
      Serial.println(", should be near 691 and closed");
    }
  }
  lidOpen = false;
  posIA = 1;
}



void loop() {
  char strBuf[16];
  int tposA = LobotSerialServoReadPosition(sb, ID1);
  int tposB = LobotSerialServoReadPosition(sb, ID2);
  
  posA = (tposA == -2048)?posA: tposA;
  posB = (tposB == -2048)?posB: tposB;
  // debug pos
  // sprintf(strBuf, "%d:%d", posA, posB);
  // Serial.println(strBuf);
  

  // ReadSignal
  if(Serial.available() > 0){
    int code = Serial.read();
    if(code%5 == 0){
      if(code == 65)  triggerOpen = true; // openlid A
      // if(code == 10)  // closelid
      if(code == 70) {
        posIA = 0;
        triggerOpen = true;
      } // 0 F
      if(code == 75) {
        posIA = 1;
        triggerOpen = true;
      } // 1 K
      if(code == 80) {
        posIA = 2;
        triggerOpen = true;
      } // 2 P
      if(code == 85) {
        posIA = 3;
        triggerOpen = true;
      } // 3 U
    }
    Serial.println(code);
  }

  if(triggerOpen && stateLid == STATELID_closed){
    stateLid = STATELID_opening;
    triggerOpen = false;
    lidOpen = true;
  }

  // long deltaTime = millis()-lastTime;

  // if(deltaTime > interval){
  //   posIA++;
  //   if(posIA>=4)posIA = 0; 
  //   lastTime = millis();
  //   lidOpen = !lidOpen;
  //   Serial.println(lidOpen);
  // }

  if(stateLid == STATELID_window){
    if(millis() - startWindow > openTimeWindow){
      stateLid = STATELID_closing;
      lidOpen = false;
    }
  }

  int targetA = posTarA[posIA];

  if(abs(targetA - posA) > tolerance){
    int deltaIdx2Tar = posIA - currentPosIA;
    if(abs(deltaIdx2Tar) == 2){
      // muter
      LobotSerialServoSetMode(sb, ID1, 1, 1000);
    }else if(deltaIdx2Tar == -1 || deltaIdx2Tar == 3){
      LobotSerialServoSetMode(sb, ID1, 1, -1000);
    }else if(deltaIdx2Tar == 1 || deltaIdx2Tar == -3){
      LobotSerialServoSetMode(sb, ID1, 1, 1000);
    }
  }else{
    currentPosIA = posIA;
    LobotSerialServoSetMode(sb, ID1, 1, 0);
  }

  if(lidOpen){
    if(posB > 1030 && posB < 1180){ // buka
      if(posB < 1131-toleranceB) LobotSerialServoSetMode(sb, ID2, 1, 190);
      else if(posB > 1131+toleranceB) LobotSerialServoSetMode(sb, ID2, 1, -190);
      else {
        LobotSerialServoSetMode(sb, ID2, 1, 0);
        if(stateLid == STATELID_opening){
          stateLid = STATELID_window;
          startWindow = millis();
        }
      }
    }else{
      LobotSerialServoSetMode(sb, ID2, 1, -1000);
    }
  }
  else{
    if(posB > 500 && posB < 760){ // tutup
      if(posB < 691-toleranceB) LobotSerialServoSetMode(sb, ID2, 1, 190);
      else if(posB > 691+toleranceB) LobotSerialServoSetMode(sb, ID2, 1, -190);
      else{
        LobotSerialServoSetMode(sb, ID2, 1, 0);
        if(stateLid == STATELID_closing) {
          stateLid = STATELID_closed;
          Serial.write(10);
        }
      }
    }else{
      LobotSerialServoSetMode(sb, ID2, 1, 1000);
    }
  }
}
