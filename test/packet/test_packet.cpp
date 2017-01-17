#include <unity.h>
#include <radio-packet.h>

#ifdef UNIT_TEST
union mway {
  unsigned char a[4];
  float b;
};

void test_read_packet(void) {
  unsigned char data[10] = "C\x01\x01\x01\x01\x01\x01\x01\x01";
  mway m;
  m.b = 1.0;
  for(int i = 0; i < 4; i++) {
    data[i + 1 + sizeof(float)] = m.a[i];
  }
  Payload radioPacket = deserialize(data);
  TEST_ASSERT_EQUAL_UINT8( (unsigned char)radioPacket.type, (unsigned char)'C');
  TEST_ASSERT_EQUAL_FLOAT( radioPacket.y, m.b);
}

int main(int argc, char** argv) {
//int main() {
  //a = 1;
  UNITY_BEGIN();
  RUN_TEST(test_read_packet);
  UNITY_END();
  return 0;
}
#endif
