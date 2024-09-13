#include <SPI.h>
#include <mcp2515.h>

#define FORWARD 0
#define BACKWARD 1

#define KLUB_ON 0x40

uint8_t buffer[256];

uint8_t sz = 0;

const uint8_t ALSNvalues[] = {0x07, 0x00, 0x01, 0x02, 0x03, 0x04};

MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS,MCP_8MHZ);
  mcp2515.setNormalMode();

  //Serial.println("Example: Write to CAN");
  showStation("MAFRANCE");
}

void showStation(char* station) {
  struct can_frame canMsg1;
  canMsg1.can_id = 1540;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = station[0];
  canMsg1.data[1] = station[1];
  canMsg1.data[2] = station[2];
  canMsg1.data[3] = station[3];
  canMsg1.data[4] = station[4];
  canMsg1.data[5] = station[5];
  canMsg1.data[6] = station[6];
  canMsg1.data[7] = station[7];
  mcp2515.sendMessage(&canMsg1);
}

void showSpeed(uint16_t speed, uint8_t direction, uint8_t acceleration) {
  struct can_frame canMsg1;
  canMsg1.can_id = 196;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0;
  canMsg1.data[1] = (speed >> 8) & 0x7F | (direction << 7);
  canMsg1.data[2] = speed & 0xFF;
  canMsg1.data[3] = 0x44;
  canMsg1.data[4] = 0x44;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = acceleration * 10;
  mcp2515.sendMessage(&canMsg1);
}


//1   Светофор
//2   Станция
//3   Опасное место
//4   Мост
//5   Переезд
//6   Платформа
//7   Тоннель
//8   Стрелка
//9   Рельсовая цепь
//10  ППУ
//11  Тупик
//12  Хвост поезда
//13  Место остановки
//14  Работают люди
//15  Усл. Разреш. Сигнал

void showParameters(uint8_t parameter, uint8_t maxSpeed, uint8_t targetSpeed, uint8_t traffic, uint16_t distance, uint8_t saut, uint8_t i) {
  struct can_frame canMsg1;
  canMsg1.can_id  = 80;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = parameter;
  canMsg1.data[1] = maxSpeed;
  canMsg1.data[2] = targetSpeed;
  canMsg1.data[3] = (distance >> 8) & 0xFF;
  canMsg1.data[4] = distance & 0xFF;
  canMsg1.data[5] = traffic;
  canMsg1.data[6] = i;
  canMsg1.data[7] = saut;
  mcp2515.sendMessage(&canMsg1);
}

void showPressure(int tm, int ur) {
  struct can_frame canMsg1;
  canMsg1.can_id  = 591;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = map(ur, 0, 100, 0, 255);
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = map(tm, 0, 100, 0, 255);
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;
  mcp2515.sendMessage(&canMsg1);
}

void showKR(uint8_t parameter) {
  struct can_frame canMsg1;
  canMsg1.can_id  = 1647;
  canMsg1.can_dlc = 1;
  canMsg1.data[0] = parameter;
  mcp2515.sendMessage(&canMsg1);
}

void showDate(uint16_t year, uint8_t month, uint8_t day, uint8_t h, uint8_t m, uint8_t s) {
  struct can_frame canMsg1;
  canMsg1.can_id  = 199;
  canMsg1.can_dlc = 7;
  canMsg1.data[0] = (year >> 8) & 0xFF;
  canMsg1.data[1] = year & 0xFF;
  canMsg1.data[2] = month;
  canMsg1.data[3] = day;
  canMsg1.data[4] = h;
  canMsg1.data[5] = m;
  canMsg1.data[6] = s;
  mcp2515.sendMessage(&canMsg1);
}

void showCoordinates(uint32_t parameter) {
  struct can_frame canMsg1;
  canMsg1.can_id  = 1541;
  canMsg1.can_dlc = 3;
  canMsg1.data[0] = (parameter >> 16) & 0xFF;
  canMsg1.data[1] = (parameter >> 8) & 0xFF;
  canMsg1.data[2] = parameter & 0xFF;
  mcp2515.sendMessage(&canMsg1);
}


int8_t processSerial() {
  uint8_t estimated = 0;
  if (sz > 255) sz = 0;
  estimated = Serial.available();
  if (estimated) {
    Serial.readBytes(&buffer[sz], estimated);
    sz += estimated;
    if (buffer[sz] = 0xD0 && buffer[sz - 1] == 0xC0 && buffer[sz - 2] == 0xB0 && buffer[sz - 3] == 0xA0) {
      if (sz >= 63) return sz - 63;
    }
  }
  return -1;
}

void loop() {
  int index = processSerial();
  if (index != -1) {
    sz = 0;
    int currentSpeed, TM, UR;
    memcpy(&currentSpeed, &buffer[index + 6], 2);
    memcpy(&TM, &buffer[index + 31], 2);
    memcpy(&UR, &buffer[index + 33], 2);
    showDate(2000, 3, 1, buffer[index + 10], buffer[index + 11], buffer[index + 12]);
    showKR(1);
    showCoordinates(0);
    delay(5);
    showParameters(KLUB_ON | 0x01, buffer[index + 0], 120, ALSNvalues[buffer[index + 4]] | (buffer[index + 5] << 4), UR, 0x00, 0);
    showPressure(TM, UR);
    showSpeed(currentSpeed, FORWARD, 0);
  }
}
