import procontroll.*;
import net.java.games.input.*;
import processing.serial.*;
import java.nio.*; 

int COMM_TIMEOUT_TIME = 1000; //one second timeout

short deadband = 125;

Serial xbeePort;
String msgReceived = "";
byte byteReceived = 0;
boolean printingMessages = false;

ControllIO controll;
ControllDevice controller;
ControllSlider leftX;
ControllSlider leftY;
ControllSlider rightX;
ControllSlider rightY;
ControllSlider analogTrigger;

byte[] MSP_HEADER = {'$', 'M', '<'};
byte[] IDENT_MSG_GET = {'$', 'M', '<', 0, 100, 100};
byte MSP_SET_RAW_RC_msgID = (byte)200;
byte ControllerMsgLength = 16;

short Roll, Pitch, Yaw, Throttle, Aux1, Aux2, Aux3, Aux4 = 0; //store each as 2 byte shorts

short[] RC_VALUES = new short[8];

void setup() {
  //start Serial port at 38400 baud and establish Multiwii connection
  
  initXbeePort();
  initXboxController();
  
  println("NOW CONTROLLING MULTIWII!\n\n");
}

void draw() {
  updateRCChannels();
  sendControllerMsg();  
  delay(20);  //send at 50Hz
}



void serialEvent(Serial port) {
  char ch = (char)port.read();
  if (printingMessages) {
    if (ch == '$') {
      print("\nRcvd Msg: ");
    }  
    print((int)ch);
    print(" ");
  }
}




/*
  Initialize xbee port at 115200 baud.  Send Identification get request to 
  quadcopter running Multiwii.  Print received Multiwii version number
*/
void initXbeePort() {
  xbeePort = new Serial(this, "COM3", 38400);  
  xbeePort.write(IDENT_MSG_GET);
}

void initXboxController() {
  controll = ControllIO.getInstance(this);
  controller = controll.getDevice("Controller (XBOX 360 For Windows)");
  leftX = controller.getSlider(0);
  leftY = controller.getSlider(1);
  rightX = controller.getSlider(2);
  rightY = controller.getSlider(3);
  analogTrigger = controller.getSlider(4);
}

void sendControllerMsg() {
  ByteBuffer controllerMsg = ByteBuffer.allocate(ControllerMsgLength).order(ByteOrder.LITTLE_ENDIAN);
  byte checksum = 0;
  checksum ^= ControllerMsgLength;
  checksum ^= MSP_SET_RAW_RC_msgID;
    
  //store RC channels into bytebuffer to send 
  for (int i = 0; i < RC_VALUES.length; i++) {
    controllerMsg.putShort(RC_VALUES[i]);
  }
  
  //update checksum
  for (int i = 0; i < controllerMsg.array().length; i++) {
    checksum ^= controllerMsg.get(i); 
  }
    
  //send data over XBee  
  xbeePort.write(MSP_HEADER);
  xbeePort.write(ControllerMsgLength);
  xbeePort.write(MSP_SET_RAW_RC_msgID);
  for (int i = 0; i < ControllerMsgLength; i++) {
    xbeePort.write(controllerMsg.get(i)); //write message data
  }
  xbeePort.write(checksum);
}

void updateRCChannels() {
  Roll = (short)map(rightY.getValue(), -1, 1, 1000, 2000);
  Pitch = (short)map(rightX.getValue(), -1, 1, 1000, 2000);
  Yaw = (short)map(analogTrigger.getValue(), 1, -1, 1000, 2000);
  Throttle = (short)map(leftX.getValue(), 1,-1, 1000, 2000);
  
  Aux1 = 1500;
  Aux2 = 1500;
  Aux3 = 1500;
  Aux4 = 1500;
  
  RC_VALUES[0] = Roll; 
  RC_VALUES[1] = Pitch; 
  RC_VALUES[2] = Yaw; 
  RC_VALUES[3] = Throttle;
  RC_VALUES[4] = Aux1;
  RC_VALUES[5] = Aux2;
  RC_VALUES[6] = Aux3;
  RC_VALUES[7] = Aux4;
  
  //implement deadband on right controller stick for roll and pitch
  for (int i = 0; i < 1; i++) {
    if (RC_VALUES[i] < (1500 + deadband) && RC_VALUES[i] > (1500 - deadband)) {
      RC_VALUES[i] = 1500;
    } else if (RC_VALUES[i] < 1500) {
      RC_VALUES[i] = (short)(RC_VALUES[i] + deadband);
    } else {
      RC_VALUES[i] = (short)(RC_VALUES[i] - deadband);
    }
  }
}
