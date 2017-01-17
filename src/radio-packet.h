#include <string.h>
typedef struct {
  char type;
  float x;
  float y;
} Payload;
Payload radioPacket;

union floatCaster {
  float f;
  unsigned char c[sizeof(float)];
};

Payload deserialize(volatile unsigned char * data) {
  Payload packet;
  packet.type = (char) data[0];
  floatCaster fc;
  for(int i = 0; i < sizeof(float); i++) {
    fc.c[i] = data[i + 1];
  }
  packet.x = fc.f;

  for(int i = 0; i < sizeof(float); i++) {
    fc.c[i] = data[i + 1 + sizeof(float)];
  }
  packet.y = fc.f;
  return packet;
}
