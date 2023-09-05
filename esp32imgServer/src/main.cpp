#include <ESP32SPISlave.h>
#include <stdio.h>
#include <FreeRTOSConfig.h>
#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <string>
#include "queue"
#include <Adafruit_MPU6050.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <SimpleKalmanFilter.h>


// pid controller definitions
#define DIST_MSK 0x1FF
#define DISTANCE_D 160
#define K_P_D_R 0.003125
#define INPUT_ANGLE -0.325 // in radians
#define THROTTLE 400

// pin Definitions
#define MICROPIN 15
#define DRIVER1 27
#define STEP1 26
#define DRIVER2 25
#define STEP2 33
// #define MPU6050_INT_STATUS 0x3A
// #define MPU6050_I2C_ADDRESS 0x68

// functions
int getParity(unsigned int n);
int checkColors(unsigned int n);
void getReadings(void * param);
// void sendPixelData(void * param);
void initWiFi();
void handleRoot();
void handleSensorData();
void handleStart();
void handleEnd();
float pWallController(int current_distance);
void writeMotors(float speedDiff);
void balanceTask(void * param);

// PID functions
float proportional_o(float er, float er2);
float integral_o(float er, float er1, float er2);
float derivative_o(float er, float er1, float er2);
float proportional_i(float er, float er2);
float integral_i(float er, float er1, float er2);
float derivative_i(float er, float er1, float er2);
float computePID(float er, float er1, float er2, float kp, float ki, float kd, float out1);


// spi stuff
ESP32SPISlave * slave = NULL;
uint8_t spi_slave_tx[3];
uint8_t spi_slave_rx[3];
TaskHandle_t * slaveHandle = NULL;

// tcp wifi stuff
const char * ssid = "Bruh";
const char * password = "qurs1567";
const int port = 8000;
const char * ip_address = "192.168.222.17";
WiFiClass * wifiObj = NULL;
WebServer * server = NULL;

// queues
// QueueHandle_t globalQueue = 0;


// motors
AccelStepper stepper1(AccelStepper::DRIVER, STEP1, DRIVER1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP2, DRIVER2);
TaskHandle_t * motorHandle = NULL;


// sensors
Adafruit_MPU6050 * mpu = NULL;
unsigned int done = 0;
sensors_event_t a;
sensors_event_t g;
sensors_event_t t;
SimpleKalmanFilter * angleFilter = NULL; 
SimpleKalmanFilter * velocFilter = NULL; 
float baseline_o;
float baseline_i;


// balancing PID
const double Ts = (1.0 / 250.0);
const double Tr = 1.0 / 600.0; 
float angAccel = 0;


// outer Loop
const double K_I_o = 1000;
const double K_D_o = 20000;//0.00003;
const double K_P_o = 250000;//0.035;

float readings_o[10];
int i_o = 0;
float value_o = 0;
float sum_o = 0;
float averaged_o = 0;

float e_o_2 = 0;
float e_o_1 = 0;
float e_o = 0;
float q_d_o_out = 0;

float theta_2 = 0;
float theta_1 = 0;


// inner Loop
const double K_I_i = 0.00;
const double K_D_i = 0.00000;
const double K_P_i = 0.9;

float readings_i[10];
int i_i = 0;
float value_i = 0;
float sum_i = 0;
float averaged_i = 0;

float e_i_2 = 0;
float e_i_1 = 0;
float e_i = 0;

float alph_d_i_out = 0;

float torque_2 = 0;
float torque_1 = 0;



void setup() {
  Serial.begin(115200);
  delay(2000);

  //////////////////////////////////////////////////////// 
  /////////////           WIFI          //////////////////
  ////////////////////////////////////////////////////////
  
  wifiObj = new WiFiClass();
  initWiFi();
  wifiObj->softAP(ssid, password);
  server = new WebServer(80);
  
  // api paths here
  server->enableCORS();
  server->on("/", handleRoot);
  server->on("/start", handleStart);
  server->on("/sensorData", handleSensorData);
  server->on("/end", handleEnd);
  server->begin(80);

  ///////////////////////////////////////////////////////////
  ////////////////          SPI         /////////////////////
  ///////////////////////////////////////////////////////////

  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
  // VSPI = CS: 5, CLK: 18, MOSI: 23, MISO: 19
  slave = new ESP32SPISlave();
  slave->setDataMode(SPI_MODE0);
  slave->begin(VSPI);

  memset(spi_slave_rx, 0, sizeof(uint8_t) * 3);
  memset(spi_slave_tx, 0, sizeof(uint8_t) * 3);

  //////////////////////////////////////////////////////////
  //////////////          FREERtos Tasks      //////////////
  //////////////////////////////////////////////////////////

  // globalQueue = xQueueCreate(3, sizeof(unsigned int));
  xTaskCreate(&getReadings, "getReadings", 2048, NULL, 1, slaveHandle);
  xTaskCreate(&balanceTask, "balanceTask", 2048, NULL, 1, motorHandle);


  //////////////////////////////////////////////////////////
  //////////////          Motors              //////////////
  //////////////////////////////////////////////////////////

  stepper1.setMaxSpeed(32000);
  stepper1.setAcceleration(16000);
  stepper2.setMaxSpeed(32000);
  stepper2.setAcceleration(16000);
  digitalWrite(MICROPIN, HIGH); // were microstepping baby
  
  //////////////////////////////////////////////////////////
  //////////////          Sensors             //////////////
  //////////////////////////////////////////////////////////
  
  mpu = new Adafruit_MPU6050();
  // mpu->setAccelerometerRange(MPU6050_RANGE_4_G); // sets a maximum range for the accelerometer
  // mpu->setGyroRange(MPU6050_RANGE_500_DEG); // sets a maximum range for the gyro
  // mpu->setFilterBandwidth(MPU6050_BAND_10_HZ); // Moving average filter, we can decrease this if the angle value jumps around alot
  // mpu->begin();
  // mpu->getEvent(&a, &g, &t); // get the accelerometer, gyro and tempereature event
  // baseline_i = g.gyro.y;
  // baseline_o = a.acceleration.roll;
  // angleFilter = new SimpleKalmanFilter(1, 1, 0.1667);
  // velocFilter = new SimpleKalmanFilter(1, 1, 0.1667);

}

float wheelSpeed = 0; // steps per second;

volatile unsigned int start = 0;
void loop() 
{
  server->handleClient();
  
  e_i_2 = e_i_1;
  e_i_1 = e_i;
  
  e_o_2 = e_o_1;
  e_o_1 = e_o;
  
  theta_2 = theta_1;
  theta_1 = q_d_o_out;
  
  torque_2 = torque_1;
  torque_1 = alph_d_i_out;

  // mpu->getEvent(&a, &g, &t); // get the accelerometer, gyro and tempereature event
  
  sum_o = sum_o - readings_o[i_o];
  readings_o[i_o] = a.acceleration.roll;
  sum_o += a.acceleration.roll;
  i_o = ( i_o + 1 ) % 10;
  averaged_o = sum_o / 10;

  // averaged_o = angleFilter->updateEstimate(averaged_o);
  e_o = INPUT_ANGLE - averaged_o;

  q_d_o_out = computePID(e_o, e_o_1, e_o_2, K_P_o, K_I_o, K_D_o, theta_1);
  wheelSpeed = (q_d_o_out * 0.1 * PI) / 16;

  // Serial.printf("/* roll: %04f, avg: %04f, rollRate: %04f result1: %04f ws: %04f */ \n", a.acceleration.roll, averaged_o, g.gyro.y, q_d_o_out, wheelSpeed); 

}

// // seq(1), col(2), len(10), x(10), y(9)
// will be turned into send changed to send
void getReadings(void * param)
{
  unsigned int currentLength = 0;
  while (1)
  {
    // wait for a transaction to be started by master
    slave->wait(spi_slave_rx, spi_slave_tx, 4); // 4 8 bit transactions are added to the queue
    
    if (slave->available())
    {
      slave->pop();
    }
    done = (spi_slave_rx[0] << 24) | (spi_slave_rx[1] << 16) | (spi_slave_rx[2] << 8) | (spi_slave_rx[3]);
    // Serial.printf("done sent: %08x \n", done);
  }
}


void initWiFi()
{
  wifiObj->begin(ssid, password);
  while (!wifiObj->isConnected())
  {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("Connected ! \n");
  Serial.println(wifiObj->localIP());
}

void handleRoot()
{
  server->send(200, "text/plain", "<div>Hi from esp32<\\div>");
}

void handleSensorData()
{
  server->send(200, "application/json", "{angle: 0.23, distance: 2}"); // imaginary sensor data
}

void handleStart()
{
  start = 1;
  server->send(200, "text/plain", "Algorithm Started");
}

void handleEnd()
{
  start = 0;
  server->send(200, "text/plain", "Algorithm Terminated");  
}


float pWallController(int current_distance)
{
  return K_P_D_R * (DISTANCE_D - current_distance);
}

// double turnWallController() TODO: Implement
// {
//   return;
// }

void writeMotors(float speedDiff)
{
  float write1 = THROTTLE * speedDiff;
  float write2 = THROTTLE * (1 - speedDiff);
  if (speedDiff >= 0)
  {
    stepper1.setSpeed(write1);
    stepper2.setSpeed(write2);

  }else
  {
    stepper1.setSpeed(write2);
    stepper2.setSpeed(write2);
  }
}

void balanceTask ( void * param )
{
  while (1)
  {

    // if (wheelSpeed >= 350)
    // {
    //   digitalWrite(MICROPIN, LOW);
    // }
    // else
    // {
    //   digitalWrite(MICROPIN, HIGH);
    // }


    stepper1.setSpeed(wheelSpeed);
    stepper2.setSpeed(wheelSpeed);

    stepper1.runSpeed();
    stepper2.runSpeed();
    

    vTaskDelay(1);
  }
}

// float proportional_o(float er, float er2)
// {
//   return K_P_o*(er-er2);
// }
// float integral_o(float er, float er1, float er2)
// {
//   return K_I_o*Ts*(er+2*er1+er2);
// }
// float derivative_o(float er, float er1, float er2)
// {
//   return (K_D_o*(er-2*er1+er2))/Ts;
// }

// float proportional_i(float er, float er2)
// {
//   return K_P_i*(er-er2);
// }
// float integral_i(float er, float er1, float er2)
// {
//   return K_I_i*Ts*(er+(2*er1)+er2);
// }
// float derivative_i(float er, float er1, float er2)
// {
//   return (K_D_i*(er-(2*er1)+er2))/Ts;
// }

float computePID(float er, float er1, float er2, float kp, float ki, float kd, float out1)
{
  return er * (kp + ki + kd) - er1 *(kp - 2 * kd) + er2 * kd + out1 * Ts;
}

