#include <IBIT.h>

#define PIN_BUTTON_A  (5)
#define PIN_BUTTON_B  (11)

uint16_t state_on_Line = 0;
uint8_t numSensor = 6;

int * _sensorPins = nullptr;
int  _NumofSensor = 0;
int * _min_sensor_values = nullptr;
int * _max_sensor_values = nullptr;
int _lastPosition = 0;
int _Sensitive  = 300;

void setup() {
  IBIT();  // initial ibit system
  setSensorPins((const int[]) {
    A0, A1, A2 , A3, A4, A5, A6, A7
  }, numSensor);
  Serial.begin(115200);
  pinMode(PIN_BUTTON_A, INPUT);
  pinMode(PIN_BUTTON_B, INPUT);

  ButtonB();

  setSensorMax((const uint16_t[]){998, 864, 1019, 1089, 961, 1056});
  setSensorMin((const uint16_t[]){34, 98, 82, 52, 67, 73});

  /*for (int i = 0; i < 3; i++) {
    for (int l = 0; l < 300; l++) {
      setCalibrate();
      delay(1);
    }
    aot(100);
    for (int l = 0; l < 300; l++) {
      setCalibrate();
      delay(1);
    } aot(100);

    for (int l = 0; l < 200; l++) {
      setCalibrate();
      delay(1);
    } aot(100);
    for (int l = 0; l < 300; l++) {
      setCalibrate();
      delay(1);
    } aot(100);
    }
    aot(100);


    for (uint8_t i = 0; i < numSensor; i++) {
    Serial.println(ReadSensorMaxValue(i));
    Serial.println(ReadSensorMinValue(i));
    delay(300);
    }*/
  ButtonB();
  Run_slow(50000, 155, 3.5, 70);


}

void loop() {

  ao();
}

void setSensorPins(const int * _pins, int _NumofSensor_)
{
  _NumofSensor = _NumofSensor_;
  _sensorPins = (int *)realloc(_sensorPins, sizeof(int) * _NumofSensor_);
  _min_sensor_values = (int *)realloc(_min_sensor_values, sizeof(int) * _NumofSensor_);
  _max_sensor_values = (int *)realloc(_max_sensor_values, sizeof(int) * _NumofSensor_);
  for (uint8_t i = 0; i < _NumofSensor_; i++)
  {
    _sensorPins[i] = _pins[i];
    _min_sensor_values[i] = 4095;
    _max_sensor_values[i] = 0;
  }

}
void setSensorMin(const uint16_t * _MinSensor)
{
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _min_sensor_values[i] = _MinSensor[i];
  }
}
void setSensorMax(const uint16_t * _MaxSensor)
{
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    _max_sensor_values[i] = _MaxSensor[i];
  }
}
void setSensitive(const uint16_t  _SensorSensitive)
{
  _Sensitive = _SensorSensitive;
}
void setCalibrate() {

  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    if (analog(_sensorPins[i]) > _max_sensor_values[i] || _max_sensor_values[i] > 1023 ) {
      _max_sensor_values[i]  = analog(_sensorPins[i]);
      if (_max_sensor_values[i] > 4095 )_max_sensor_values[i] = 4095;
    }
  }
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    if (analog(_sensorPins[i]) < _min_sensor_values[i] || _min_sensor_values[i] == 0) {
      _min_sensor_values[i] = analog(_sensorPins[i]);
      if (_min_sensor_values[i] < 0) _min_sensor_values[i] = 0;
    }
  }

}
int ReadSensorMinValue(uint8_t _Pin) {
  return _min_sensor_values[_Pin];
}
int ReadSensorMaxValue(uint8_t _Pin) {
  return _max_sensor_values[_Pin];
}
int readline()
{
  bool onLine = false;
  long avg = 0;
  long sum = 0;
  for (uint8_t i = 0; i < _NumofSensor; i++)
  {
    long value = map(analog(_sensorPins[i]), _min_sensor_values[i], _max_sensor_values[i], 1000, 0);                                                                            // จากนั้นก็เก็บเข้าไปยังตัวแป value
    if (value > _Sensitive) {
      onLine = true;
    }
    if (value > 50)
    {
      avg += (long)value * (i * 1000);
      sum += value;
    }
  }
  if (!onLine)
  {
    if (_lastPosition < (_NumofSensor - 1) * 1000 / 2)
    {
      return 0;
    }
    else
    {
      return (_NumofSensor - 1) * 1000;
    }
  }
  _lastPosition = avg / sum;
  return _lastPosition;
}

void ao() {
  motor_stop(ALL);
}

void aot(int _time) {
  motor_stop(ALL);
  delay(_time);
}

void ButtonB() {
  while (true) { // จนกว่าจะกดปุ่ม
    if (! digitalRead(PIN_BUTTON_B)) {
      Serial.println("Button B Pressed");
      break;
    }
  }
}

//------------------------------------------Function PID Track------------------------------------------

void Run_slow(int delay_timer, int base_speed, float Kp, float Kd ) {
  //float Kp = 4 ;
  float Ki = 0;
  //float Kd = 60;
  uint16_t setpoint;
  float present_position;
  float errors = 0;
  float output = 0;
  float integral ;
  float derivative ;
  float previous_error ;
  long timer = millis();
  do {
    //int base_speed = 50;
    present_position = readline() / ((numSensor - 1) * 10) ;
    setpoint = 75.0;
    errors = setpoint - present_position;
    integral = integral + errors ;
    derivative = (errors - previous_error) ;
    output = Kp * errors + Ki * integral + Kd * derivative;
    int max_output = 200;
    previous_error = errors;
    if (output > max_output )output = max_output;
    else if (output < -max_output)output = -max_output;
    motor(1, base_speed - output);
    motor(2, base_speed + output);
    delay(1);

  } while (millis() - timer < delay_timer);
}
