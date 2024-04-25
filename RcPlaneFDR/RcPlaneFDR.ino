/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x64 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPSPlus.h>
#include <ITG3200.h>
#include <Adafruit_HMC5883_U.h>
#include <SimpleKalmanFilter.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_BMP280 bmp; // I2C baro
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); //accel

static const uint32_t GPSBaud = 115200;
TinyGPSPlus gps;
HardwareSerial uart0(0);

ITG3200 gyro;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

SimpleKalmanFilter baroFilter = SimpleKalmanFilter(3, 3, 0.1);

float axcal = 0;
float aycal = 0;
float azcal = 0;
float gxcal = 0;
float gycal = 0;
float gzcal = 0;



void loadgpsdata(){
  while (uart0.available() > 0)
    if (gps.encode(uart0.read()))
  
    if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}



void setup() {
  Wire.begin();
  Serial.begin(115200);
  uart0.begin(GPSBaud); //start interfaces

  Serial.println("init"); 

  Serial.println("_");
  Serial.println("│");
  Serial.println(F("├>ssd1306 init"));
  Serial.println("│");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.print(" SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();


  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Welcome");

  display.display();
  delay(2000);

  Serial.println(F("├>BMP280 init"));
  Serial.println("│");
  unsigned status;
  status = bmp.begin(0x76);
  

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("├>adxl345 init");
  Serial.print("│");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print(" Ooops, no ADXL345 detected ... Check your wiring!");
  }
  Serial.println();

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);

  float axtmp = 0;
  float aytmp = 0;
  float aztmp = 0;
  float calCount = 50;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("hold still");
  display.display();
  delay(2000);

  for(int i = 0; i <= calCount; i++){
    sensors_event_t event; 
    accel.getEvent(&event);
    axtmp = axtmp + event.acceleration.x;
    aytmp = aytmp + event.acceleration.y;
    aztmp = aztmp + event.acceleration.z;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(axtmp/i);
    display.println("x off");
    display.print(aytmp/i);
    display.println("y off");
    display.print(aztmp/i);
    display.println("z off");
    display.display();

    delay(100);
  }

  axcal = axtmp/calCount;
  aycal = aytmp/calCount;
  azcal = aztmp/calCount;
  delay(2000);

  Serial.println("├>itg 3200 init");
  Serial.println("│");
  gyro.init();
  gyro.zeroCalibrate(200, 10); //sample 200 times to calibrate and it will take 200*10ms

  Serial.println("├>HMC5883L init");
  Serial.print("│");
  if(!mag.begin())
    {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.print(" Ooops, no HMC5883 detected ... Check your wiring!");
    }
  Serial.println();


  Serial.println("└>gps init");
  while (uart0.available() > 0)
    if (gps.encode(uart0.read()))
  
    if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.print(F(" No GPS detected: check wiring."));
    while(true);
  }

  Serial.println("init done");

  int sats = 0;
  int threshold = 0;
  while(sats<threshold){
    loadgpsdata();
    sats = gps.satellites.value();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("sats");
    display.print(sats);
    display.print("/");
    display.print(threshold);
    display.display();
    Serial.println("waiiting for sats");
    Serial.println(gps.satellites.value());
    delay(100);
  }

}

void loop() {
  loadgpsdata();

  sensors_event_t accelEvent; 
  accel.getEvent(&accelEvent);


  float totAccel = sqrt(sq(abs(accelEvent.acceleration.x))+sq(abs(accelEvent.acceleration.y))+sq(abs(accelEvent.acceleration.z)));

  Serial.print(baroFilter.updateEstimate(bmp.readAltitude()));
  Serial.print(",");
  Serial.println(bmp.readAltitude());

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(gps.speed.mph());
  display.println(" mph");
  display.print(gps.satellites.value());
  display.println(" sats");
  display.print(totAccel);
  display.println(" ms-1");
  display.display();

  delay(100);
}
