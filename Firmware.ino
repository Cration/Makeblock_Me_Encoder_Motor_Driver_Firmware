/*************************************************************************
* File Name          : EncoderMotorDriverFirmware.ino
* Author             : Ander
* Updated            : Ander
* Version            : V1.0.0
* Date               : 3/22/2014
* Description        : Firmware for Encoder DC Motor's Driver base on Arduino Uno.
* License            : CC-BY-SA 3.0
* Copyright (C) 2014 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/

#include "Encoder.h"
#include "PID.h"
#include <Wire.h>
#include <EEPROM.h>

byte PIN_ENCODER_SIG1[2] = {2, 3};
byte PIN_ENCODER_SIG2[2] = {10, 12};
byte PIN_MOTOR_DIR1[2] = {A1, 9};
byte PIN_MOTOR_DIR2[2] = {A0, 8};
byte PIN_MOTOR_PWM[2] = {6, 5};

boolean DEBUG = false;

#define SPEED_MODE 0
#define DISTANCE_MODE 1
#define TIME_MODE 2
#define FINISH_MODE 3

#define ENCODER_1 0
#define ENCODER_2 1

#define EEPROM_I2C 1
#define EEPROM_PID_SPEED 2
#define EEPROM_PID_DISTANCE 26
#define EEPROM_RATIO 54
#define EEPROM_COUNTER 62
Encoder encoders[2] = {Encoder(PIN_ENCODER_SIG1[0], PIN_ENCODER_SIG2[0]), Encoder(PIN_ENCODER_SIG1[1], PIN_ENCODER_SIG2[1])};

float _motor_P_S[2] = {3.0, 3.0}; //{3.4,3.4};
float _motor_I_S[2] = {0.02, 0.02};
float _motor_D_S[2] = {0.0, 0.0};

float _motor_P_D[2] = {0.15, 0.15};
float _motor_I_D[2] = {0.0054, 0.0054};
float _motor_D_D[2] = {0, 0};

float _ratio[2]     = {75, 75};    
byte _counters[2]   = {24, 24};
long _motor_time[2] = {0, 0};
long _motor_last[2] = {0, 0};
long _position[2]   = {0, 0};    
float _speed[2]     = {0, 0};        
float _max_speed[2], _prevSpeed[2], _scale_speed[2] = {0.12, 0.12};
boolean _motor_flag[2] = {true, true};
boolean _isReset[2]    = {false, false};
boolean _running[2]    = {false, false};
int32_t _prevInput[2];
int _maxPWM[2];
int _mode[2] = {FINISH_MODE, FINISH_MODE};

double _time, _prevTime;

int _pwmInit = 2;
int _limitPID = 30 - _pwmInit;
int _sampleTime = 50;

int _PID_Counter_Limit;
long debugTime = 0;

double _PID_input[2] = {0, 0}, _PID_output[2] = {0, 0}, _PID_setpoint[2] = {0, 0}, _PID_Speed[2] = {0, 0}, _PWM_output[2] = {0, 0};
int32_t _PID_Dist[2] = {0, 0};
PID _myPIDs[2] = {PID(&_PID_input[0], &_PID_output[0], &_PID_setpoint[0], 0, 0, 0, DIRECT), PID(&_PID_input[1], &_PID_output[1], &_PID_setpoint[1], 0, 0, 0, DIRECT)};

union
{
    byte b[4];
    float fVal;
    long lVal;
} u;
uint8_t rxBuf[32];
void setup()
{
    Serial.begin(9600);
    //  EEPROM.write(0,0xff);
    if(EEPROM.read(0) == 0xff)
    {
        EEPROM.write(0, 0xA1);
        setEncoderPID(_motor_P_D[ENCODER_1], 0, ENCODER_1, DISTANCE_MODE);
        setEncoderPID(_motor_I_D[ENCODER_1], 1, ENCODER_1, DISTANCE_MODE);
        setEncoderPID(_motor_D_D[ENCODER_1], 2, ENCODER_1, DISTANCE_MODE);
        setEncoderPID(_motor_P_S[ENCODER_1], 0, ENCODER_1, SPEED_MODE);
        setEncoderPID(_motor_I_S[ENCODER_1], 1, ENCODER_1, SPEED_MODE);
        setEncoderPID(_motor_D_S[ENCODER_1], 2, ENCODER_1, SPEED_MODE);
        setEncoderPID(_motor_P_D[ENCODER_2], 0, ENCODER_2, DISTANCE_MODE);
        setEncoderPID(_motor_I_D[ENCODER_2], 1, ENCODER_2, DISTANCE_MODE);
        setEncoderPID(_motor_D_D[ENCODER_2], 2, ENCODER_2, DISTANCE_MODE);
        setEncoderPID(_motor_P_S[ENCODER_2], 0, ENCODER_2, SPEED_MODE);
        setEncoderPID(_motor_I_S[ENCODER_2], 1, ENCODER_2, SPEED_MODE);
        setEncoderPID(_motor_D_S[ENCODER_2], 2, ENCODER_2, SPEED_MODE);
        setEncoderRatio(75, ENCODER_1);
        setCounters(24, ENCODER_1);
        setEncoderRatio(75, ENCODER_2);
        setCounters(24, ENCODER_2);
        setI2CAddress(0x1);
    }
    _myPIDs[ENCODER_1].SetOutputLimits(-_limitPID, _limitPID);
    _myPIDs[ENCODER_1].SetSampleTime(_sampleTime);
    _myPIDs[ENCODER_1].SetMode(AUTOMATIC);

    _myPIDs[ENCODER_2].SetOutputLimits(-_limitPID, _limitPID);
    _myPIDs[ENCODER_2].SetSampleTime(_sampleTime);
    _myPIDs[ENCODER_2].SetMode(AUTOMATIC);

    getAllEncoderPID();
    getEncoderRatio();
    getEncoderCounter();
    _prevTime = 0;
    Serial.print("I2C Address:0x");
    Serial.println(getI2CAddress(), HEX);
    Wire.begin(getI2CAddress());
    Wire.onReceive(receiveEvent); // register receive event
    Wire.onRequest(requestEvent); // register request event
    //runMotorWithAngleAndSpeed(360,1,ENCODER_1);
    //runMotorWithAngleAndSpeed(360,1,ENCODER_1);
    //runMotorWithTurns(10,ENCODER_1);
    //runMotorWithSpeedAndTime(10,5000,ENCODER_1);
}
void receiveEvent(int howMany)
{
    byte i = 0;
    uint8_t c = 0;
    while( Wire.available())
    {
        c = Wire.read(); // receive byte as a character]
        rxBuf[i] = c;
        i++;
    }
    u.fVal = encoders[rxBuf[2]].read() * 360.0 / _counters[rxBuf[2]] / _ratio[rxBuf[2]];
}
void requestEvent()
{
    i2cCommand(rxBuf);
}
void setI2CAddress(byte addr)
{
    EEPROM.write(EEPROM_I2C, addr);
}
byte getI2CAddress()
{
    pinMode(7, INPUT);
    pinMode(11, INPUT);
    pinMode(13, INPUT);
    pinMode(A6, INPUT);
    digitalWrite(7, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(13, HIGH);
    int A6_flag = 0;
    if (analogRead(A6) > 10)
    {
        A6_flag = 1;
    }
    return  0x10 - (digitalRead(7) + (digitalRead(11) << 1) + (digitalRead(13) << 2) + (A6_flag << 3)) + (EEPROM.read(EEPROM_I2C));
}
void setEncoderPID(float val, byte type, byte encoder, byte mode)
{
    if(mode == DISTANCE_MODE)
    {
        if(type == 0)
        {
            _motor_P_D[encoder] = val;
        }
        else if(type == 1)
        {
            _motor_I_D[encoder] = val;
        }
        else
        {
            _motor_D_D[encoder] = val;
        }
        if(encoder == ENCODER_1)
        {
            if(type < 3)
            {
                writeFloat(EEPROM_PID_DISTANCE + type * 4, val);
            }
        }
        else
        {
            if(type < 3)
            {
                writeFloat(EEPROM_PID_DISTANCE + 12 + type * 4, val);
            }
        }
    }
    if(mode == SPEED_MODE)
    {
        if(type == 0)
        {
            _motor_P_S[encoder] = val;
        }
        else if(type == 1)
        {
            _motor_I_S[encoder] = val;
        }
        else
        {
            _motor_D_S[encoder] = val;
        }
        if(encoder == ENCODER_1)
        {
            if(type < 3)
            {
                writeFloat(EEPROM_PID_SPEED + type * 4, val);
            }
        }
        else
        {
            if(type < 3)
            {
                writeFloat(EEPROM_PID_SPEED + 12 + type * 4, val);
            }
        }
    }
}
float getEncoderPID(byte type, byte encoder, byte mode)
{
    if(mode == DISTANCE_MODE)
    {
        if(type == 0)
        {
            return _motor_P_D[encoder];
        }
        else if(type == 1)
        {
            return _motor_I_D[encoder];
        }
        else
        {
            return _motor_D_D[encoder];
        }
    }
    if(mode == SPEED_MODE)
    {
        if(type == 0)
        {
            return _motor_P_S[encoder];
        }
        else if(type == 1)
        {
            return _motor_I_S[encoder];
        }
        else
        {
            return _motor_D_S[encoder];
        }
    }
}
void getAllEncoderPID()
{
    _motor_P_D[ENCODER_1] = readFloat(EEPROM_PID_DISTANCE);
    _motor_I_D[ENCODER_1] = readFloat(EEPROM_PID_DISTANCE + 4);
    _motor_D_D[ENCODER_1] = readFloat(EEPROM_PID_DISTANCE + 8);
    _motor_P_D[ENCODER_2] = readFloat(EEPROM_PID_DISTANCE + 12);
    _motor_I_D[ENCODER_2] = readFloat(EEPROM_PID_DISTANCE + 16);
    _motor_D_D[ENCODER_2] = readFloat(EEPROM_PID_DISTANCE + 20);
    _motor_P_S[ENCODER_1] = readFloat(EEPROM_PID_SPEED);
    _motor_I_S[ENCODER_1] = readFloat(EEPROM_PID_SPEED + 4);
    _motor_D_S[ENCODER_1] = readFloat(EEPROM_PID_SPEED + 8);
    _motor_P_S[ENCODER_2] = readFloat(EEPROM_PID_SPEED + 12);
    _motor_I_S[ENCODER_2] = readFloat(EEPROM_PID_SPEED + 16);
    _motor_D_S[ENCODER_2] = readFloat(EEPROM_PID_SPEED + 20);
}
void i2cCommand(uint8_t *cmd)
{
    if(cmd[1] > 0x30 && cmd[1] < 0x40)
    {
        if(_motor_flag[cmd[2]] == false)
        {
            return;
        }
    }
    if(cmd[0] == 0x21)
    {
        setI2CAddress(cmd[1]);
        return;
    }
    switch(cmd[1])
    {
    case 0x22:
    {
        setEncoderRatio(cmd[4], cmd[3] == ENCODER_1 ? ENCODER_1 : ENCODER_2);
    }
    break;
    case 0x23:
    {
        setCounters(cmd[4], cmd[3] == ENCODER_1 ? ENCODER_1 : ENCODER_2);
    }
    break;
    case 0x24:
    {
        setEncoderPID(readFloatFromArray(cmd + 5), cmd[3], cmd[4], cmd[2]);
    }
    break;
    case 0x31:
    {
        runMotorWithAngleAndSpeed(readLongFromArray(cmd + 3), readFloatFromArray(cmd + 7), cmd[2], false);
    }
    break;
    case 0x32:
    {
        runMotorWithAngleAndSpeed(readLongFromArray(cmd + 3), readFloatFromArray(cmd + 7), cmd[2], true);
    }
    break;
    case 0x33:
    {
        runMotorWithSpeedAndTime(readFloatFromArray(cmd + 3), readLongFromArray(cmd + 7), cmd[2]);
    }
    break;
    case 0x41:
    {
        setCommandFlag(cmd[3], cmd[2]);
    }
    break;
    case 0x51:
    {
        byte r[7] = {0x91, cmd[1], cmd[2], 0, 0, 0, 0};
        int i;
        u.fVal = _speed[cmd[2]] * 100.0 / 3.0;
        for(i = 0; i < 4; i++)
        {
            r[3+i] = u.b[i];
        }
        Wire.write(r, 7);
    }
    break;
    case 0x52:
    {
        byte r[7] = {0x91, cmd[1], cmd[2], 0, 0, 0, 0};
        int i;
        for(i = 0; i < 4; i++)
        {
            r[3+i] = u.b[i];
        }
        Wire.write(r, 7);
    }
    break;
    case 0x53:
    {
        byte r[9] = {0x91, cmd[1], cmd[2], cmd[3], cmd[4], 0, 0, 0, 0};
        int i;
        u.fVal = getEncoderPID(cmd[3], cmd[2], cmd[4]);
        for(i = 0; i < 4; i++)
        {
            r[5+i] = u.b[i];
        }
        Wire.write(r, 9);
    }
    break;
    case 0x54:
    {
        resetEncoder(cmd[2]);
        byte r[4] = {0x91, 0x11, cmd[1], 0x1};
        Wire.write(r, 4);
    }
    break;
    case 0x55:
    {
        DEBUG = !DEBUG;
        byte r[4] = {0x91, 0x11, cmd[1], 0x1};
        Wire.write(r, 4);
    }
    default:
        break;
    }
    if(cmd[1] < 0x50)
    {
        byte r[4] = {0x91, 0x11, cmd[1], 0x1};
        Wire.write(r, 4);
    }
}
float readFloatFromArray(byte *arr)
{
    int i;
    for(i = 0; i < 4; i++)
    {
        u.b[i] = arr[i];
    }
    return u.fVal;
}
long readLongFromArray(byte *arr)
{
    int i;
    for(i = 0; i < 4; i++)
    {
        u.b[i] = arr[i];
    }
    return u.lVal;
}
float readFloat(int addr)
{
    int i;
    for(i = 0; i < 4; i++)
    {
        u.b[i] = EEPROM.read(addr + i);
    }
    return u.fVal;
}
long readLong(int addr)
{
    int i;
    for(i = 0; i < 4; i++)
    {
        u.b[i] = EEPROM.read(addr + i);
    }
    return u.lVal;
}
void writeFloat(int addr, float val)
{
    u.fVal = val;
    int i;
    for(i = 0; i < 4; i++)
    {
        EEPROM.write(addr + i, u.b[i]);
    }
}
void writeLong(int addr, long val)
{
    u.lVal = val;
    int i;
    for(i = 0; i < 4; i++)
    {
        EEPROM.write(addr + i, u.b[i]);
    }
}

void loop()
{
    if(_isReset[0] == true)
    {
        _isReset[0] = false;
        encoders[0].write(0);
    }
    if(_isReset[1] == true)
    {
        _isReset[1] = false;
        encoders[1].write(0);
    }
    runEncoder();
}
void runEncoder()
{
    _time = micros() - _prevTime;
    if(_time < 40000 && _time > 0)
    {
        return;
    }
    _prevTime = micros();
    runEncoder(ENCODER_1);
    runEncoder(ENCODER_2);
}

void setEncoderRatio(float ratio, int encoder)
{
    _ratio[encoder] = ratio;
    writeFloat(EEPROM_RATIO + encoder * 4, ratio);
}
void getEncoderRatio()
{
    _ratio[ENCODER_1] = readFloat(EEPROM_RATIO);
    _ratio[ENCODER_2] = readFloat(EEPROM_RATIO + 4);
    if(_ratio[ENCODER_1] == 0)
    {
        setEncoderRatio(75, ENCODER_1);
    }
    if(_ratio[ENCODER_2] == 0)
    {
        setEncoderRatio(75, ENCODER_2);
    }
    Serial.print("ratio1:");
    Serial.print(_ratio[ENCODER_1]);
    Serial.print(" ratio2:");
    Serial.println(_ratio[ENCODER_2]);
}
void setCounters(byte counters, int encoder)
{
    _counters[encoder] = counters;
    _PID_Counter_Limit = counters * 6;
    Serial.println(counters);
    EEPROM.write(EEPROM_COUNTER + encoder, counters);
}
void getEncoderCounter()
{
    _counters[ENCODER_1] = EEPROM.read(EEPROM_COUNTER);
    _counters[ENCODER_2] = EEPROM.read(EEPROM_COUNTER + 1);
    _PID_Counter_Limit =  _counters[ENCODER_1] * 4;
    Serial.print("counter1:");
    Serial.print(_counters[ENCODER_1]);
    Serial.print(" counter2:");
    Serial.println(_counters[ENCODER_2]);
}
int32_t getDistance(int encoder)
{
    return _PID_Dist[encoder] - _position[encoder];
}
void resetEncoder(int encoder)
{
    _mode[encoder] = FINISH_MODE;
    _PID_Speed[encoder] = 0;
    _PID_setpoint[encoder] = 0;
    _PID_Dist[encoder] = 0;
    _isReset[encoder] = true;
}
void setEncoderPosition(int encoder, long position)
{
    encoders[encoder].write(position);
}
void runMotorWithAngle(int encoder)
{
    _mode[encoder] = FINISH_MODE;
    _PID_setpoint[encoder] = _PID_Dist[encoder];
    _myPIDs[encoder].SetTunings(_motor_P_D[encoder], _motor_I_D[encoder], _motor_D_D[encoder]);
    _max_speed[encoder] = 10 * _ratio[encoder] * _counters[encoder] / 1000.0;
    _maxPWM[encoder] = 40;
}
void runMotorWithAngleAndSpeed(long degree, float speed, int encoder, bool isAbsolute)
{
    _running[encoder] = true;
    _mode[encoder] = DISTANCE_MODE;
    speed = abs(speed);
    float dist = degree * _ratio[encoder] * _counters[encoder] / 360.0;
    if(isAbsolute)
    {
        _PID_Speed[encoder] = (dist - _position[encoder] < 0) ? -speed : speed;
    }
    else
    {
        _PID_Speed[encoder] = degree < 0 ? -speed : speed;
    }
    _PID_Dist[encoder] = dist + (isAbsolute ? 0 : _PID_Dist[encoder]);
    _PID_setpoint[encoder] = _PID_Speed[encoder] * _ratio[encoder] * _counters[encoder] / 1000.0;
    _myPIDs[encoder].SetTunings(_motor_P_S[encoder], _motor_I_S[encoder], _motor_D_S[encoder]);
    _max_speed[encoder] = _PID_Speed[encoder] * _ratio[encoder] * _counters[encoder] / 1000.0;
    _scale_speed[encoder] = 0.2 / (speed > 1 ? (speed - 0.5) : (speed * speed + 0.4));
    _myPIDs[encoder].reset();

    _PID_input[encoder] = 0;
    _PWM_output[encoder] = 0;
    _maxPWM[encoder] = 250;
}
void runMotorWithTurns(float turns, int encoder)
{
    runMotorWithAngleAndSpeed(turns * 360.0, 3, encoder, false);
}
void runMotorWithSpeedAndTime(float speed, long time, int encoder)
{
    _mode[encoder] = TIME_MODE;
    _motor_time[encoder] = time;
    if(time == 0)
    {
        _mode[encoder] = SPEED_MODE;
    }
    _motor_last[encoder] = millis();
    _running[encoder] = true;
    _PID_Speed[encoder] = speed;
    _PID_setpoint[encoder] = _PID_Speed[encoder] * _ratio[encoder] * _counters[encoder] / 1000.0;
    _myPIDs[encoder].reset();
    _myPIDs[encoder].SetTunings(_motor_P_S[encoder], _motor_I_S[encoder], _motor_D_S[encoder]);
    _max_speed[encoder] = 10.0 * _ratio[encoder] * _counters[encoder] / 1000.0;
    _maxPWM[encoder] = 250;
}
void runEncoder(int encoder)
{
    _position[encoder] = encoders[encoder].read();
    _speed[encoder] = (_position[encoder] - _prevInput[encoder]) * 1000.0 / _time;
    float currentSpeed = 0;
    _PID_input[encoder] = _mode[encoder] == FINISH_MODE ? _position[encoder] : _speed[encoder];

    if(_running[encoder] == true)
    {
        _myPIDs[encoder].Compute();
    }
    int dir = getDistance(encoder) > 0 ? 1 : -1;
    if(_mode[encoder] == DISTANCE_MODE)
    {
        currentSpeed = _speed[encoder];
        long distance = abs(getDistance(encoder));
        _PID_Speed[encoder] = (_PID_Dist[encoder] - _position[encoder] < 0) ? -abs(_PID_Speed[encoder]) : abs(_PID_Speed[encoder]);
        _PID_setpoint[encoder] = _PID_Speed[encoder] * _ratio[encoder] * _counters[encoder] / 1000.0;
        if((distance < abs(_PID_Speed[encoder])*_PID_Counter_Limit))
        {
            runMotorWithAngle(encoder);
        }
        _PWM_output[encoder] += _PID_output[encoder];
        if(abs(currentSpeed) > abs(_max_speed[encoder]) + 1)
        {
            _PWM_output[encoder] = 0;
            runMotorWithAngle(encoder);
        }
        if(encoder == ENCODER_1 && DEBUG)
        {
            Serial.print(currentSpeed);
            Serial.print(",");
            Serial.println(distance);
        }
    }
    else if(_mode[encoder] == FINISH_MODE)
    {
        dir = 0;
        currentSpeed = _speed[encoder];
        long distance = (getDistance(encoder));
        _PWM_output[encoder] = distance * _scale_speed[encoder];
        setCommandFlag(true, encoder);
        if(encoder == ENCODER_1 && DEBUG)
        {
            Serial.print(currentSpeed);
            Serial.print(",");
            Serial.println(distance);
        }
        _prevSpeed[encoder] = currentSpeed;
        if(abs(currentSpeed) < 0.03 && distance < 40)
        {
            _PWM_output[encoder] = 0;
            stopMotor(encoder);
            _running[encoder] = false;
        }
    }
    else
    {
        currentSpeed = _speed[encoder];
        _PWM_output[encoder] += _PID_output[encoder];
        if(encoder == ENCODER_1 && DEBUG)
        {
            Serial.print(currentSpeed);
            Serial.print(",");
            Serial.println(_PWM_output[encoder]);
        }
        if(_mode[encoder] == TIME_MODE)
        {
            if(millis() - _motor_last[encoder] >= _motor_time[encoder])
            {
                _PID_Dist[encoder] = encoders[encoder].read();
                stopMotor(encoder);
                runMotorWithAngleAndSpeed(0, _PID_Speed[encoder], encoder, false);
            }
        }
    }
    _prevInput[encoder] = _position[encoder];

    _PWM_output[encoder] = _PWM_output[encoder] > _maxPWM[encoder] ? _maxPWM[encoder] : (_PWM_output[encoder] < -_maxPWM[encoder] ? -_maxPWM[encoder] : _PWM_output[encoder]);
    if(abs(_PWM_output[encoder]) > 100 && abs(currentSpeed) == 0)
    {
        stopMotor(encoder);
        _mode[encoder] = FINISH_MODE;
        _PID_setpoint[encoder] = encoders[encoder].read();
    }
    else
    {
        if(_PWM_output[encoder] > _pwmInit)
        {
            digitalWrite(PIN_MOTOR_DIR1[encoder], HIGH);
            digitalWrite(PIN_MOTOR_DIR2[encoder], LOW);
            analogWrite(PIN_MOTOR_PWM[encoder], _pwmInit + _PWM_output[encoder]);
        }
        else if(_PWM_output[encoder] < -_pwmInit)
        {
            digitalWrite(PIN_MOTOR_DIR1[encoder], LOW);
            digitalWrite(PIN_MOTOR_DIR2[encoder], HIGH);
            analogWrite(PIN_MOTOR_PWM[encoder], _pwmInit - _PWM_output[encoder]);
        }
        else
        {
            stopMotor(encoder);
        }
    }
}

void stopMotor(int encoder)
{
    digitalWrite(PIN_MOTOR_DIR1[encoder], HIGH);
    digitalWrite(PIN_MOTOR_DIR2[encoder], HIGH);
    analogWrite(PIN_MOTOR_PWM[encoder], 0);
}

void setCommandFlag(boolean flag, int encoder)
{
    _motor_flag[encoder] = flag;
}

void setCurrentDistance(long distance, int encoder)
{
    _PID_Dist[encoder] = distance;
    encoders[encoder].write(distance);
}
