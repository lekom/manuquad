 import procontroll.*;
import net.java.games.input.*;
import processing.serial.*;
import java.nio.*; 

boolean RunningOnMacSide = true;

int horizonModeButtonNum = 9;
int acroModeButtonNum = 8;
int altitudeHoldModeButtonNum = 11;
int gpsHoldModeButtonNum = 12;


int COMM_TIMEOUT_TIME = 1000; //one second timeout
short deadband = 75;

Serial xbeePort;
String msgReceived = "";
byte byteReceived = 0;
boolean printingMessages = false;
int msgID = 4; //location of message ID in msgReceived 
int minMsgLength = 6;
int gpsMsgLength = minMsgLength + 16;
int analogMsgLength = minMsgLength + 7;
int altitudeMsgLength = minMsgLength + 6;
int attitudeMsgLength = minMsgLength + 6;
int analogMsgID = 110;
int gpsMsgID = 106;
int altitudeMsgID = 109;
int attitudeMsgID = 108;

ControllIO controll;
ControllDevice controller;
ControllSlider leftX;
ControllSlider leftY;
ControllSlider rightX;
ControllSlider rightY;
ControllSlider analogTrigger;
ControllSlider analogTriggerR; //for mac side...

ControllButton horizonModeButton;
ControllButton acroModeButton;
ControllButton altitudeHoldModeButton;
ControllButton gpsHoldModeButton;

boolean lastHorizonModeButtonState = false;
boolean horizonModeButtonPressed = false;

boolean lastAltitudeHoldModeButtonState = false;
boolean altitudeHoldModeButtonPressed = false;

boolean lastAcroModeButtonState = false;
boolean acroModeButtonPressed = false; 

boolean lastGpsHoldModeButtonState = false;
boolean gpsHoldModeButtonPressed = false;

int currentFlightMode = 0;
int acroMode = 0;
int horizonMode = 1;
int altitudeHoldMode = 2;
int gpsHoldMode = 3;

byte[] MSP_HEADER = {'$', 'M', '<'};
byte[] IDENT_MSG_GET = {'$', 'M', '<', 0, 100, 100};
byte[] ANALOG_MSG_GET = {'$', 'M', '<', 0, 110, 110};
byte[] GPS_MSG_GET = {'$','M','<',0,106,106};
byte[] ALTITUDE_MSG_GET = {'$','M','<',0,109,109};
byte[] ATTITUDE_MSG_GET = {'$','M','<',0,108,108};

byte MSP_SET_RAW_RC_msgID = (byte)200;
byte ControllerMsgLength = 16;

short Roll, Pitch, Yaw, Throttle, Aux1, Aux2, Aux3, Aux4 = 0; //store each as 2 byte shorts

short[] RC_VALUES = new short[8];

long displayUpdateDelay = 200;
long lastDisplayUpdate = 0;
long lastAnalogGetTime = 0;
long lastGPSGetTime = 0;
long lastAltitudeGetTime = 0;
long lastAttitudeGetTime = 0;
long getAnalogDelay = 1000; //retrieve battery level every second
long getAttitudeDelay = 200;
long getGPSDelay = 200; 
long getAltitudeDelay = 200; 
long now = 0;

int vbatt = 0;
int minBatt = 140; //14 volts

//GPS Data
boolean hasGPSfix = false;
byte numGPSSats = 0;
int currLatitude = 0;
int currLongitude = 0;
short altitudeGPS = 0;
short speedGPS = 0;
short groundCourseGPS = 0;

//Altitude Data
int altitudeBaro = 0;
short varioBaro = 0;

//Attitude Data
short angx = 0;
short angy = 0;
short heading = 0;

boolean redOn = false;

void setup() {
  //start Serial port at 38400 baud and establish Multiwii connection
  initXbeePort();
  initXboxController();
  
  println("NOW CONTROLLING MULTIWII!\n\n");
  
  horizonModeButton = controller.getButton(horizonModeButtonNum); //A button on windows, 'UP' control pad on mac
  acroModeButton = controller.getButton(acroModeButtonNum);
  altitudeHoldModeButton = controller.getButton(altitudeHoldModeButtonNum);
  gpsHoldModeButton = controller.getButton(gpsHoldModeButtonNum);
  
  size(800,800);
  textSize(32);
}

void draw() {  
  now = millis();
  if ((now - lastAnalogGetTime) >= getAnalogDelay) {
    sendAnalogGet();
    lastAnalogGetTime = now;
  }
  
  if ((now - lastGPSGetTime) >= getGPSDelay) {
    sendGPSGet();
    lastGPSGetTime = now; 
  }
  
  if ((now - lastAltitudeGetTime) >= getAltitudeDelay) {
    sendAltitudeGet();
    lastAltitudeGetTime = now;
  }
  
  if ((now - lastAttitudeGetTime) >= getAttitudeDelay) {
    sendAttitudeGet();
    lastAttitudeGetTime = now;
  }
    
  updateRCChannels();
  sendControllerMsg();  
  delay(10);  //send at 50Hz
  //println(Yaw);
  
  if ((now - lastDisplayUpdate) >= displayUpdateDelay) {
    updateDisplay();
    lastDisplayUpdate = now;
  }
}


void updateDisplay() {
  textSize(32);
  if (vbatt < minBatt) {
      if (!redOn) {
      background(255,0,0);
      redOn = true;
    } else {
      background(0,0,0);
      redOn = false;
    }  
    text("BATTERY LOW! (" + ((float)vbatt)/10 + "V)", 5,30);  
  } else {
    background(0,255,0);
    text("Battery Level: " + ((float)vbatt)/10 + "V", 5,30);
  }

  if (hasGPSfix) {
    text("GPS Coordinates: " + (float)(currLatitude) / 10000000 + "  " + (float)(currLongitude) / 10000000, 5, 70);    
  } else {
    text("NO GPS FIX... yet", 5, 70);
  }

  text("Altitude: baro: " + altitudeBaro + "  GPS: " + altitudeGPS, 5, 110);
  text("heading:  " + heading, 5, 150);  
}

void serialEvent(Serial port) {
  char ch = (char)port.read();
  
  //receiving next message, handle previous message
  if (ch == '$') { 
    if (msgReceived.length() >= minMsgLength) {
      if (msgReceived.charAt(msgID) == analogMsgID && msgReceived.length() >= analogMsgLength) {
        handleAnalogMsg();
      } else if (msgReceived.charAt(msgID) == gpsMsgID && msgReceived.length() >= gpsMsgLength) {
        handleGPSMsg(); 
      } else if (msgReceived.charAt(msgID) == altitudeMsgID && msgReceived.length() >= altitudeMsgLength) {
        handleAltitudeMsg();
      } else if (msgReceived.charAt(msgID) == attitudeMsgID && msgReceived.length() >= attitudeMsgLength) {
        handleAttitudeMsg();  
      }
    } 
    msgReceived = "";
  }
  
  msgReceived += ch;
  
  if (printingMessages) {
    if (ch == '$') {
      print("\nRcvd Msg: ");
    }  
    print((int)ch);
    print(" ");
  }
}

void sendAttitudeGet() {
  xbeePort.write(ATTITUDE_MSG_GET);
}

void sendAnalogGet() {
  xbeePort.write(ANALOG_MSG_GET);
}

void sendGPSGet() {
  xbeePort.write(GPS_MSG_GET);
}

void sendAltitudeGet() {
  xbeePort.write(ALTITUDE_MSG_GET);
}

void handleAttitudeMsg() {
  angx = (short)((short)msgReceived.charAt(6)<<8 | (short)msgReceived.charAt(5));
  angy = (short)((short)msgReceived.charAt(8)<<8 | (short)msgReceived.charAt(7));
  heading = (short)((short)msgReceived.charAt(10)<<8 | (short)msgReceived.charAt(9));
}

void handleAltitudeMsg() {
  altitudeBaro = (int)msgReceived.charAt(8)<<24 | (int)msgReceived.charAt(7)<<16 | (int)msgReceived.charAt(6)<<8 | (int)msgReceived.charAt(5);;
  varioBaro = (short)((short)msgReceived.charAt(10)<<8 | (short)msgReceived.charAt(9));
}

void handleGPSMsg() {
  hasGPSfix = (msgReceived.charAt(5) == 1);
  
  if (hasGPSfix) {
    numGPSSats = (byte)msgReceived.charAt(6);
    currLatitude = (int)msgReceived.charAt(10)<<24 | (int)msgReceived.charAt(9)<<16 | (int)msgReceived.charAt(8)<<8 | (int)msgReceived.charAt(7);
    currLongitude = (int)msgReceived.charAt(14)<<24 | (int)msgReceived.charAt(13)<<16 | (int)msgReceived.charAt(12)<<8 | (int)msgReceived.charAt(11);
    altitudeGPS = (short)((short)msgReceived.charAt(16)<<8 | (short)msgReceived.charAt(15));
    speedGPS = (short)((short)msgReceived.charAt(18)<<8 | (short)msgReceived.charAt(17));
    groundCourseGPS = (short)((short)msgReceived.charAt(20)<<8 | (short)msgReceived.charAt(19));
  } 
}

void handleAnalogMsg() {
  vbatt = (int)msgReceived.charAt(5);
  //prin\ tln("battery voltage:  " + vbatt);
}

/*
  Initialize xbee port at 38400 baud.  Send Identification get request to 
  quadcopter running Multiwii. 
*/
void initXbeePort() {
  xbeePort = new Serial(this, "/dev/tty.usbserial-A601EO8E", 38400);  
  xbeePort.write(IDENT_MSG_GET);
}

void initXboxController() {
  controll = ControllIO.getInstance(this);

  if (RunningOnMacSide) {
    controller = controll.getDevice("Wireless 360 Controller"); //for running on mac
    leftX = controller.getSlider(1);
    leftY = controller.getSlider(0);
    rightX = controller.getSlider(3);
    rightY = controller.getSlider(2);
    analogTrigger = controller.getSlider(4);
    analogTriggerR = controller.getSlider(5);
  } else {
    controller = controll.getDevice("Controller (XBOX 360 For Windows)"); //for running on windows
    leftX = controller.getSlider(0);
    leftY = controller.getSlider(1);
    rightX = controller.getSlider(2);
    rightY = controller.getSlider(3);
    analogTrigger = controller.getSlider(4);
  }
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
  
  /* handle mode buttons */
  horizonModeButtonPressed = horizonModeButton.pressed();
  altitudeHoldModeButtonPressed = altitudeHoldModeButton.pressed();
  acroModeButtonPressed = acroModeButton.pressed();
  gpsHoldModeButtonPressed = gpsHoldModeButton.pressed();
  
  if (horizonModeButtonPressed && !lastHorizonModeButtonState) {
    currentFlightMode = horizonMode;  
    println("Horizon Mode Enabled");
  } else if (acroModeButtonPressed && !lastAcroModeButtonState) {
    currentFlightMode = acroMode;
    println("Acro Mode Enabled");
  } else if (altitudeHoldModeButtonPressed && !lastAltitudeHoldModeButtonState) {
    currentFlightMode = altitudeHoldMode;
    println("Altitude Hold Mode Enabled");
  } else if (gpsHoldModeButtonPressed && !lastGpsHoldModeButtonState) {
    currentFlightMode = gpsHoldMode;
    println("GPS Hold Mode Enabled");
  }
  
  lastHorizonModeButtonState = horizonModeButtonPressed;
  lastAcroModeButtonState = acroModeButtonPressed;
  lastAltitudeHoldModeButtonState = altitudeHoldModeButtonPressed;
  lastGpsHoldModeButtonState = gpsHoldModeButtonPressed;
  
  Roll = (short)map(rightY.getValue(), -1, 1, 1250, 1750);
  Pitch = (short)map(rightX.getValue(), -1, 1, 1750, 1250);
  Throttle = (short)map(leftX.getValue(), 1,-1, 1000, 2000);
  
  if (RunningOnMacSide) {
    short leftTrig = (short)map(analogTrigger.getValue(), 1, -1, 500, 0);
    short rightTrig = (short)map(analogTriggerR.getValue(), 1, -1, -500, 0);
    Yaw = (short)(1500 + leftTrig + rightTrig);
  } else {
    Yaw = (short)map(analogTrigger.getValue(), 1, -1, 2000, 1000);
  }
  
  if (currentFlightMode == horizonMode) {
    Aux1 = 2000; 
    Aux2 = 1500;
    Aux3 = 1500;
  } else if (currentFlightMode == altitudeHoldMode) {
    Aux2 = 2000; //hold altitude
    Aux1 = 2000; //hold horizon
    Aux3 = 1500;
  } else if (currentFlightMode == gpsHoldMode) {
    Aux1 = 2000; //hold horizon
    Aux2 = 2000; //hold Altitude
    Aux3 = 2000; //hold GPS
  } else if (currentFlightMode == acroMode) {
    Aux1 = 1500;
    Aux2 = 1500; 
    Aux3 = 1500;
  }
  
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
  for (int i = 0; i <= 1; i++) {
    if (RC_VALUES[i] < (1500 + deadband) && RC_VALUES[i] > (1500 - deadband)) {
      RC_VALUES[i] = 1500;
    } else if (RC_VALUES[i] < 1500) {
      RC_VALUES[i] = (short)(RC_VALUES[i] + deadband);
    } else {
      RC_VALUES[i] = (short)(RC_VALUES[i] - deadband);
    }
  }
}
