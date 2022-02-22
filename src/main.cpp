/*
This example takes range measurements with the VL53L1X and displays additional 
details (status and signal/ambient rates) for each measurement, which can help
you determine whether the sensor is operating normally and the reported range is
valid. The range is in units of mm, and the rates are in units of MCPS (mega 
counts per second).
*/

#include <Wire.h>
//#include "VL53L1X.h"
#include "SparkFun_VL53L1X.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal.h>
#include <LcdProgressBar.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <LiquidMenu.h>
#include <Preferences.h>

/*
VL53L1X sensor;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  //Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(40000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(100);
}

void loop()
{
  sensor.read();
  
  Serial.print("range: ");
  Serial.print(sensor.ranging_data.range_mm);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);
  
  Serial.println();
}

*/




SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

//Store distance readings to get rolling average
#define ENTER 27
#define UP 12
#define DOWN 14
int Width; //in case of cartesian 
int Depth;//in case of cartesian	   
int heigh;	//heigh of tank 1 in cm	, from sensor to base	inside  
int Diameter;
bool alarmed; 
float Area; //the area of the tank
const int MAX_DISTANCE = 200;                       //max distance to measure
int Litres1, LiqLevel, prev=0, delta, maxlitres, menu_count, Distance1;
long prevtime;
String message;
Bounce bouncer_Enter = Bounce();
Bounce bouncer_Up = Bounce();
Bounce bouncer_Minus = Bounce();

#define HISTORY_SIZE 8
int history[HISTORY_SIZE+1];
byte historySpot;
long lastReading = 0, menuTimer=0;
long lastDistance = 0;
float newDistance;
int maxDistance = 0;
#define LOOPTIME 50
boolean readingValid = false;
long validCount = 0;
    
// Set password to "" for open networks.
char ssid[] = "COSMOTE-189DDC_AC";		                                            //local wifi network SSID
char pass[] = "UXYebdfUddddKqAq";			                                //local network password
char auth[] = "0eGWRUn2X8QvaWEpplH9UzT3pd-qh8LO";                                     // You should get Authority Token in your email.  
SimpleTimer timer, timerMenu;                                                                   //config timer
LiquidCrystal lcd(2,4,16,17,5,18); //LiquidCrystal(rs, enable,d0, d1, d2, d3);
LcdProgressBar lpg(&lcd, 1, 16);
void set_dim();
void rotate_value();
Preferences prefs;
char* progress=(char *)malloc(17);

///*************************************************************
typedef enum {
  CARTESSIAN,
  CYLINDER
} Coordinate_type;
Coordinate_type coordinates=CARTESSIAN;

//*******************connect check
typedef enum {
  CONNECT_TO_WIFI,
  AWAIT_WIFI_CONNECTION,
  CONNECT_TO_BLYNK,
  AWAIT_BLYNK_CONNECTION,
  MAINTAIN_CONNECTIONS,
  AWAIT_DISCONNECT
} CONNECTION_STATE;

CONNECTION_STATE connectionState;
uint8_t connectionCounter;

void ConnectionHandler(void) {
  switch (connectionState) {
  case CONNECT_TO_WIFI:
    BLYNK_LOG("Connecting to %s.", ssid);
    WiFi.begin(ssid, pass);
    connectionState = AWAIT_WIFI_CONNECTION;
    connectionCounter = 0;
    break;

  case AWAIT_WIFI_CONNECTION:
    if (WiFi.status() == WL_CONNECTED) {
      BLYNK_LOG("Connected to %s", ssid);
      connectionState = CONNECT_TO_BLYNK;
    }
    else if (++connectionCounter == 50) {
      BLYNK_LOG("Unable to connect to %s. Retry connection.", ssid);
      WiFi.disconnect();
      connectionState = AWAIT_DISCONNECT;
      connectionCounter = 0;
    }
    break;

  case CONNECT_TO_BLYNK:
    BLYNK_LOG("Attempt to connect to Blynk server.");
    Blynk.config(auth, IPAddress(139, 59, 206, 133), 80);
    Blynk.connect();
    connectionState = AWAIT_BLYNK_CONNECTION;
    connectionCounter = 0;
    break;

  case AWAIT_BLYNK_CONNECTION:
    if (Blynk.connected()) {
      BLYNK_LOG("Connected to Blynk server.");
      connectionState = MAINTAIN_CONNECTIONS;
    }
    else if (++connectionCounter == 50) {
      BLYNK_LOG("Unable to connect to Blynk server. Retry connection.");
      Blynk.disconnect();
      WiFi.disconnect();
      connectionState = AWAIT_DISCONNECT;
      connectionCounter = 0;
    }
    break;

  case MAINTAIN_CONNECTIONS:
    if (WiFi.status() != WL_CONNECTED) {
      BLYNK_LOG("Wifi connection lost. Reconnect.");
      Blynk.disconnect();
      WiFi.disconnect();
      connectionState = AWAIT_DISCONNECT;
      connectionCounter = 0;
    }
    else  if (!Blynk.connected()) {
      BLYNK_LOG("Blynk server connection lost. Reconnect.");
      Blynk.disconnect();
      connectionState = CONNECT_TO_BLYNK;
    }
    else {
      Blynk.run();
    }
    break;

  case AWAIT_DISCONNECT:
    if (++connectionCounter == 10) {
      connectionState = CONNECT_TO_WIFI;
    }
    break;
  }
}
//*******************************************************************************************
//************Blync sync all*************
BLYNK_CONNECTED() {
    Blynk.syncAll();
}
//***************************************

//****************MENU Definition******************
LiquidLine welcome_line0(0, 0, "Liquid: ",Litres1,"ltr");
LiquidLine welcome_line1(0, 1, "Level: ", LiqLevel,"cm");
LiquidLine welcome_line2(0, 1, "Empty: ", Distance1,"cm");
LiquidLine setting(0, 1, "<Set_dim>");
LiquidScreen welcome_screen(welcome_line0,welcome_line1, welcome_line2, setting);

LiquidLine line21(0, 0, "Set_Dim");
LiquidLine line22(0, 1, "Coord_Sys:",coordinates);
LiquidLine line23(0, 1, "<Conf>");
LiquidLine line24(0, 1, "<Esc>");
LiquidScreen screen2(line21, line22, line23);


LiquidLine line31(0, 0, "Cartessian");
LiquidLine line32(0, 1, "Height: ",heigh,"cm");
LiquidLine line33(0, 1, "Depth: ",Depth,"cm");
LiquidLine line34(0, 1, "Width: ",Width,"cm");
LiquidLine line35(0, 1, "<Save>");
LiquidLine line36(0, 1, "<Esc>");
LiquidScreen screen3(line31, line32, line33, line34);

LiquidLine line41(0, 0, "Cylinder");
LiquidLine line42(0, 1, "Height: ",heigh,"cm");
LiquidLine line43(0, 1, "Diameter: ",Diameter,"cm");
LiquidLine line44(0, 1, "<Save>");
LiquidLine line45(0, 1, "<Esc>");
LiquidScreen screen4(line41, line42, line43);

LiquidMenu menu(lcd,welcome_screen);
//**************************************************************

//********Buttons handle**********
void  buttonsCheck() {
  uint8_t push_cnt=0;
  //Serial.println("entered button check");
  try{
    //Serial.println("entered try");
    bouncer_Up.update();
   if (bouncer_Up.fell()) 
	 {
     menuTimer=millis();
        menu.call_function(1);
        while(bouncer_Up.read()==LOW && ++push_cnt <3) 
        {
        delay(1000);
        bouncer_Up.update();
        }
        push_cnt=0;
        while(bouncer_Up.read()==LOW) 
        {
        menu.call_function(1); 
        delay(50);
        bouncer_Up.update();
        }
    menu.softUpdate();
	  }
    bouncer_Minus.update();
  if (bouncer_Minus.fell())
  {
    menuTimer=millis();
		  menu.call_function(2);
      while(bouncer_Minus.read()==LOW && ++push_cnt <3) 
      {delay(1000);
      bouncer_Minus.update();
      }
      push_cnt=0;
      while(bouncer_Minus.read()==LOW) 
      {
      menu.call_function(2); 
      delay(50);
      bouncer_Minus.update();
      }
    menu.softUpdate();
    }
    //menu.previous_screen();*/
    
	//}
  bouncer_Enter.update();
	if (bouncer_Enter.fell()) {
		// Switches focus to the next line.
    menuTimer=millis();
    if(((menu.get_currentScreen()==&screen2)&&(menu.get_focusedLine()==3))||((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==5))||((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==4)))
    {
      menu.call_function(3);
    }
    else if(((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==4))||((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==3)))
    {
      menu.call_function(3);
    }
    else
    {
      //Serial.println("entered ENTER check");
      menu.switch_focus();
      
    } 
    menu.softUpdate();
	}
  }
  catch(std::exception e) {
  Serial.println(e.what());
 }
}

//***************************************
//***************************************
void idle_function(){
;
}
void nextScreen()
{
  //Serial.printf("current screen = %d \n",menu.get_currentScreen());
 // Serial.printf("focused line = %d \n",menu.get_focusedLine());
  if((menu.get_currentScreen()==&welcome_screen)&&(menu.get_focusedLine()==3))
    { 
      menu.change_screen(&screen2);
      //Serial.printf("current screen = %d \n",menu.get_currentScreen());
    }
  else if((menu.get_currentScreen()==&screen2)&&(menu.get_focusedLine()==2))
  {
    if(coordinates==CARTESSIAN)
      menu.change_screen(&screen3);
    else if(coordinates==CYLINDER)
      menu.change_screen(&screen4);
    else
      menu.change_screen(&screen3);
  }
  else if((menu.get_currentScreen()==&screen2)&&(menu.get_focusedLine()==3))
  {
    menu.switch_focus();
    menu.change_screen(&welcome_screen);
  }
  else if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==5))
  {
    menu.switch_focus();
    menu.change_screen(&welcome_screen);
  }
  else if((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==4))
  {
    menu.switch_focus();
    menu.change_screen(&welcome_screen);
  }
   
}

void nextLine()
{
  menu.switch_focus(true);
}

void prevLine()
{
  menu.switch_focus(false);
}


//***********************************
void incr_dimension()
{
  if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==1))
  {
    heigh++;
    if(heigh==401) heigh=0;
  }
  if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==2))
  {
    Depth++;
    if(Depth==201) Depth=0;
  }
  if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==3))
  {
    Width++;
    if(Width==201) Depth=0;
  }
  if((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==1))
  {
    heigh++;
    if(heigh==401) heigh=0;
  }
  if((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==2))
  {
    Diameter++;
    if(Diameter==201) heigh=0;
  }
}

void decr_dimension()
{
  if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==1))
  {
    heigh--;
    if(heigh==0) heigh=400;
  }
  if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==2))
  {
    Depth--;
    if(Depth==0) Depth=200;
  }
  if((menu.get_currentScreen()==&screen3)&&(menu.get_focusedLine()==3))
  {
    Width--;
    if(Width==0) Depth=200;
  }
  if((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==1))
  {
    heigh--;
    if(heigh==0) heigh=400;
  }
  if((menu.get_currentScreen()==&screen4)&&(menu.get_focusedLine()==2))
  {
    Diameter--;
    if(Diameter==0) heigh=200;
  }
}
//**********************************
void set_coordinates()
{
  switch(coordinates){
    case CARTESSIAN:
      coordinates=CYLINDER;
      break;
    case CYLINDER:
      coordinates=CARTESSIAN;
      break;
    default:
      coordinates=CARTESSIAN;

  }
}

void save_func()
{
   prefs.putInt("heigh", heigh);
   prefs.putInt("Width", Width);
   prefs.putInt("Depth", Depth);
   prefs.putInt("Diameter", Diameter);
   //if(EEPROM.read(4)!=coordinates) EEPROM.write(4,coordinates);
   prefs.putInt("Coord", (int)coordinates);
   menu.switch_focus();
   menu.switch_focus();
   menu.change_screen(&welcome_screen);
}

//******Print messages to lcd**********
void LCDwrite(String msg )
{
 LcdProgressBar lpg(&lcd, 1, 16);
  //LCD_progress_bar (0, WaterDepth1, 0, MAX_DISTANCE);
  lpg.setMinValue(0);
  lpg.setMaxValue(MAX_DISTANCE);
  delay(200);
  lpg.draw(LiqLevel);
  //lcd.clear();
    // go to row 1 column 0, note that this is indexed at 0
    //for(int i=0;i<16;i++) lcd.write(1022);
   lcd.setCursor(0,0); 
   lcd.print(msg);
   //lcd.print (String(Litres1)+" Liters");
   
}

//****************Refresh Menu******************
void refresh_menu()
{
  if (millis()-menuTimer>30000)
  {
    if(millis()-menuTimer>30000&&millis()-menuTimer<30100)
    lcd.clear();
    message="FuelVol: "+String(Litres1)+"Ltr";
    LCDwrite(message); 
  }
  else 
  {
    menu.softUpdate();
  }
  
  /* if(menu.get_currentScreen()==&welcome_screen) 
  {
    //menu.set_focusPosition(Position::LEFT);
    lpg.draw(LiqLevel);
  }*/
}

//***********************************

//************************************************************************************************




void sendSensorReadings()
{
	//LiqLevel = heigh - Distance1;							        //calculate the depth of the water
	//Litres1 = (Area * LiqLevel) / 1000;	                         //calculate the volume of the water in litres
 // maxlitres = (Area * MAX_DISTANCE) / 1000;
  delta=prev-Litres1; 
  prev=Litres1;
//***********SEND INFO TO BLYNK
Blynk.virtualWrite(V1, LiqLevel);                                //send depth to Blynk server
Blynk.virtualWrite(V2, Litres1);                                            //send litres to Blynk server
Blynk.virtualWrite(V3, Litres1 ); 
Blynk.virtualWrite(V4, delta);
//send litres to Blynk server. for vertical level widget & chart, 
//scaled to 1/10 as Blynk only goes up to 9999 and we need up to 16000
if (Litres1<=(maxlitres/8))
{
  if (!alarmed) Blynk.email("g.j.stamelos@gmail.com","subject: Tank is almost empty", "Your Tank has just "+String(Litres1)+" liters. Care of filling it up" );
  Blynk.virtualWrite(V0, 255);
  alarmed=true;
}
  else if (Litres1>=(maxlitres/7))
{
  if (alarmed) Blynk.email("g.j.stamelos@gmail.com","subject: Tank is level restored", "Your Tank has now "+String(Litres1)+" liters." );
  Blynk.virtualWrite(V0, 0);
  alarmed=false;
}

//************************* can be commented out, test use only
//Serial.println();
//Serial.println();
//Serial.println("Tank 1 water distance: " + String(Distance1));   //print depth
//Serial.println("Tank 1 water depth: " + String(LiqLevel));  //print depth
//Serial.println("Tank 1 Litres: " + String(Litres1));                //print litres
//***********************************************   

}

//********************Take measure******************
void take_measure()
{
    if(coordinates==0)
    Area=Width*Depth; //the area of the cartessian tank
  else if(coordinates==1)
    Area=(float)3.14*pow((float)(Diameter/2),2);
  //Serial.printf("Area(cm): %2f\n",Area);
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  //lastReading = millis();

  history[historySpot] = distance;
  if (historySpot++ == HISTORY_SIZE)
    historySpot = 0;
long avgDistance = 0;
  for (int x = 0; x < HISTORY_SIZE; x++)
    avgDistance += history[x];

  avgDistance /= HISTORY_SIZE;

  //Every loop let's get a reading
  newDistance = distance / 10; //Go get distance in cm
  int signalRate = distanceSensor.getSignalRate();
  if (signalRate < 10)
  {
    readingValid = false;
    validCount = 0;
  }
  else
  {
    validCount++;
    if (avgDistance > maxDistance)
      maxDistance = avgDistance;
  }

  if (validCount > 10)
    readingValid = true;
  Distance1=readingValid?avgDistance/10:maxDistance/10;
  Serial.print("Distance(cm): ");
  Serial.print(readingValid?avgDistance/10:maxDistance/10);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);
  LiqLevel = heigh - Distance1;							        //calculate the depth of the water
	Litres1 = (Area * LiqLevel) / 1000;	                         //calculate the volume of the water in litres
  maxlitres = (Area * MAX_DISTANCE) / 1000;
}




//************************setup*********************


void setup(void)
{
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");
  prefs.begin("values",false);
  //prefs.clear();
  heigh=prefs.getInt("heigh",200);
  Width=prefs.getInt("Width",100);
  Depth=prefs.getInt("Depth",50);
  Diameter=prefs.getInt("Diameter",100);
  coordinates=(Coordinate_type)prefs.getInt("Coord",CARTESSIAN);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  for (int x = 0; x < HISTORY_SIZE; x++)
    history[x] = 0;

 /*  if(EEPROM.read(0)==0) 
    {
      set_dim(); //Dimentions initialization if this is first time
      EEPROM.write(0,1);
    }
    */
  pinMode(15,OUTPUT);
  digitalWrite(15,LOW);
  pinMode(13, OUTPUT);                                                //LED D7
  pinMode(ENTER,INPUT_PULLUP);
  bouncer_Enter.attach(ENTER);
  bouncer_Enter.interval(5);
  pinMode(UP,INPUT_PULLUP);
  bouncer_Up.attach(UP);
  bouncer_Up.interval(5);
  pinMode(DOWN,INPUT_PULLUP);
  bouncer_Minus.attach(DOWN);
  bouncer_Minus.interval(5);
	timer.setInterval(3000, sendSensorReadings);		// Setup a function to be called every n seconds
  timer.setInterval(10,buttonsCheck);
  timer.setInterval(300, ConnectionHandler);
  timer.setInterval(200,refresh_menu);
  timer.setInterval(1000,take_measure);
  menuTimer=millis();
  connectionState = CONNECT_TO_WIFI;
  //Blynk.begin(auth, ssid, pass);  
  lcd.begin(16, 2);
  lcd.clear();
  //lcd.print("Tank Level meter");

  // go to row 1 column 0, note that this is indexed at 0
  //lcd.setCursor(0,0); 
 // lcd.print ("LCD with ESP32");
  lpg.setMinValue(0);
  lcd.clear();
  lpg.setMaxValue(MAX_DISTANCE);
  delay(2000);
  lpg.draw(MAX_DISTANCE);
  delay(2000);
  welcome_line0.attach_function(1,idle_function);
  setting.attach_function(1, nextScreen);
  setting.attach_function(2, nextScreen);
  line22.attach_function(1, set_coordinates);
  line22.attach_function(2, set_coordinates);
  line23.attach_function(1, nextScreen);
  line23.attach_function(2, nextScreen); 
  line24.attach_function(3, nextScreen); 

  line32.attach_function(1, incr_dimension);
  line32.attach_function(2, decr_dimension);
  line33.attach_function(1, incr_dimension);
  line33.attach_function(2, decr_dimension);
  line34.attach_function(1, incr_dimension);
  line34.attach_function(2, decr_dimension); 
  line35.attach_function(3, save_func);
  line36.attach_function(3, nextScreen);
 
  line42.attach_function(1, incr_dimension);
  line42.attach_function(2, decr_dimension);
  line43.attach_function(1, incr_dimension);
  line43.attach_function(2, decr_dimension);
  line44.attach_function(3, save_func);
  line45.attach_function(3, nextScreen);

  menu.init();
  menu.add_screen(screen2);
	menu.add_screen(screen3);
	menu.add_screen(screen4);
  screen2.add_line(line24);
  screen3.add_line(line35);
  screen3.add_line(line36);
  screen4.add_line(line44);
  screen4.add_line(line45);

  welcome_screen.set_displayLineCount(2);
  screen2.set_displayLineCount(2);
  screen3.set_displayLineCount(2);
  screen4.set_displayLineCount(2);
}

void loop(void)
{
  timer.run(); 

}


///******************************
void set_dim()
{
  
  if(EEPROM.read(1)!=heigh) EEPROM.write(1,heigh);
  if(EEPROM.read(2)!=Width) EEPROM.write(2,Width);
  if(EEPROM.read(3)!=Depth) EEPROM.write(3,Depth);
  EEPROM.commit();
}
void rotate_value()
{
;

}