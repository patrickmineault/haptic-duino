//******************************************************************************
//****** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE ****
//******************************************************************************
// for Feather 32u4 Radio
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4

#define NETWORKID     100  // The same on all nodes that talk to each other
#define SENDER        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to the hardware version of the radio on your Feather
//RF69_433MHZ | RF69_868MHZ | RF69_915MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "dbbe604988162c28" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
