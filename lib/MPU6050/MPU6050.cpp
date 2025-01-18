/**
 * @author Crispin Mukalay
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * MPU-6050 triple-axis MEMS gyroscope and triple-axis MEMS accelerometer.
 *
 * Datasheet:
 *
 * https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 */

/**
 * Includes
 */
#include "MPU6050.h"
#include <string>

MPU6050::MPU6050(I2C &i2c) : i2c_(i2c) {
    //initialize(void);
}

MPU6050::MPU6050(PinName sda, PinName scl) : i2c_(*(new I2C(sda, scl))) {
    //400kHz, fast mode.
    i2c_.frequency(400000);
}

void MPU6050::selfTest(uint8_t* TestReadings) {
    
    char buffer[4];
    uint8_t Xa1, Xa2, Ya1, Ya2, Za1, Za2;
    
    buffer[0] = SingleByteRead(SELF_TEST_X_REG);
    buffer[1] = SingleByteRead(SELF_TEST_Y_REG);
    buffer[2] = SingleByteRead(SELF_TEST_Z_REG);
    buffer[3] = SingleByteRead(SELF_TEST_A_REG); 
    
    //Split the bytes
    Xa1 = (int)buffer[0] >> 5 & 0x07;
    Ya1 = (int)buffer[1] >> 5 & 0x07;
    Za1 = (int)buffer[2] >> 5 & 0x07;
    
    Xa2 = (int)buffer[3] & 0x30;
    Ya2 = (int)buffer[3] & 0x0C;
    Ya2 = (int)buffer[3] & 0x03;
    
    TestReadings[0] = Xa1 | Xa2;
    TestReadings[1] = Ya1 | Ya2;
    TestReadings[2] = Za1 | Za2;
    TestReadings[3] = (int)buffer[0] & 0x1F;
    TestReadings[4] = (int)buffer[1] & 0x1F;
    TestReadings[5] = (int)buffer[2] & 0x1F;
}

void MPU6050::setSampleRate(uint8_t Divider){
    
    SingleByteWrite(SMPLRT_DIV_REG, Divider);
}

uint8_t MPU6050::getSampleRate(void){
    
    return SingleByteRead(SMPLRT_DIV_REG);
}

void MPU6050::setFSYNCConfig(uint8_t Conf){
    
    uint8_t conf_L;
    uint8_t conf_H;
    uint8_t conf_HL;
        
    conf_L = SingleByteRead(CONFIG_REG) & 0x07;
    conf_H = Conf << 3;
    conf_HL = conf_H | conf_L;
    
    SingleByteWrite(CONFIG_REG, conf_HL);   
}

uint8_t MPU6050::getFSYNCConfig(void){
    
    return (int)SingleByteRead(CONFIG_REG) >> 3 & 0x07;
}

void MPU6050::setDLPFConfig(uint8_t Conf){
    
    uint8_t conf_L;
    uint8_t conf_H;
    uint8_t conf_HL;
    
    conf_L = Conf;    
    conf_H = SingleByteRead(CONFIG_REG) & 0x38;
    conf_HL = conf_H | conf_L;
    
    SingleByteWrite(CONFIG_REG, conf_HL);  
    
}

uint8_t MPU6050::getDLPFConfig(void){
    
    return (int)SingleByteRead(CONFIG_REG) & 0x07;
}

void MPU6050::setGyroConfig(uint8_t GyroST, uint8_t Scale){
    
    SingleByteWrite(GYRO_CONFIG_REG, GyroST | Scale);
}

uint8_t MPU6050::getGyroConfig(void){
    
    return (int)SingleByteRead(GYRO_CONFIG_REG);
}

void MPU6050::setAccelConfig(uint8_t AccST, uint8_t Scale){
    
    SingleByteWrite(ACCEL_CONFIG_REG, AccST | Scale);
}

uint8_t MPU6050::getAccelConfig(void){
    
    return (int)SingleByteRead(ACCEL_CONFIG_REG);
}

void MPU6050::setFIFO_Enable(uint8_t Setting){
    
    SingleByteWrite(FIFO_EN_REG , Setting);
}

uint8_t MPU6050::getFIFO_Enable(void){
    
    return (int)SingleByteRead(FIFO_EN_REG );
}

void MPU6050::readAccel(uint16_t* AccReadings) {
    
    char ACCEL_OUT_buffer[6];
    
    ACCEL_OUT_buffer[0] = SingleByteRead(ACCEL_XOUT_H_REG);
    ACCEL_OUT_buffer[1] = SingleByteRead(ACCEL_XOUT_L_REG);
    ACCEL_OUT_buffer[2] = SingleByteRead(ACCEL_YOUT_H_REG);
    ACCEL_OUT_buffer[3] = SingleByteRead(ACCEL_YOUT_L_REG);
    ACCEL_OUT_buffer[4] = SingleByteRead(ACCEL_ZOUT_H_REG);
    ACCEL_OUT_buffer[5] = SingleByteRead(ACCEL_ZOUT_L_REG);
    
    AccReadings[0] = (int)ACCEL_OUT_buffer[0] << 8 | (int)ACCEL_OUT_buffer[1];
    AccReadings[1] = (int)ACCEL_OUT_buffer[2] << 8 | (int)ACCEL_OUT_buffer[3];
    AccReadings[2] = (int)ACCEL_OUT_buffer[4] << 8 | (int)ACCEL_OUT_buffer[5];
}

void MPU6050::readTemp(uint16_t* TempReadings) {
    
    char TEMP_OUT_buffer[2];
    
    TEMP_OUT_buffer[0] = SingleByteRead(TEMP_OUT_H_REG );
    TEMP_OUT_buffer[1] = SingleByteRead(TEMP_OUT_L_REG );
    
    TempReadings[0] = (int)TEMP_OUT_buffer[0] << 8 | (int)TEMP_OUT_buffer[1];
}

// Vitória das Dores - Lê os valores brutos (raw) do giroscópio (LSB) - Adicionado para ler os dados brutos.
void MPU6050::readGyroRaw(int16_t* gyroReadings) {
    char gyro_buffer[6];

    multiByteRead(GYRO_XOUT_H_REG, gyro_buffer, 6);

    gyroReadings[0] = (int16_t)(gyro_buffer[0] << 8 | gyro_buffer[1]);
    gyroReadings[1] = (int16_t)(gyro_buffer[2] << 8 | gyro_buffer[3]);
    gyroReadings[2] = (int16_t)(gyro_buffer[4] << 8 | gyro_buffer[5]);
}

void MPU6050::readGyro(uint16_t* GyroReadings) {
    
    char GYRO_OUT_buffer[6];
    
    GYRO_OUT_buffer[0] = SingleByteRead(GYRO_XOUT_H_REG);
    GYRO_OUT_buffer[1] = SingleByteRead(GYRO_XOUT_L_REG);
    GYRO_OUT_buffer[2] = SingleByteRead(GYRO_YOUT_H_REG);
    GYRO_OUT_buffer[3] = SingleByteRead(GYRO_YOUT_L_REG);
    GYRO_OUT_buffer[4] = SingleByteRead(GYRO_ZOUT_H_REG);
    GYRO_OUT_buffer[5] = SingleByteRead(GYRO_ZOUT_L_REG);
    
    GyroReadings[0] = (int)GYRO_OUT_buffer[0] << 8 | (int)GYRO_OUT_buffer[1];
    GyroReadings[1] = (int)GYRO_OUT_buffer[2] << 8 | (int)GYRO_OUT_buffer[3];
    GyroReadings[2] = (int)GYRO_OUT_buffer[4] << 8 | (int)GYRO_OUT_buffer[5];   
}

void MPU6050::sigPathReset(uint8_t ResVal){
    
    SingleByteWrite(SIGNAL_PATH_RESET_REG, ResVal);
}

void MPU6050::setUserCtl(uint8_t Settings){
    
    SingleByteWrite(USER_CTRL_REG, Settings);
}

uint8_t MPU6050::getUserCtl(void){
    
    return (int)SingleByteRead(USER_CTRL_REG);
}

void MPU6050::setPowerCtl_1(uint8_t DevRes, uint8_t Sleep, uint8_t Cycle, uint8_t Temp, uint8_t Clock){
    
    uint8_t powerSetting;
    
    powerSetting = DevRes       | Sleep;
    powerSetting = powerSetting | Cycle;
    powerSetting = powerSetting | Temp;
    powerSetting = powerSetting | Clock;
    
    SingleByteWrite(PWR_MGMT_1_REG, powerSetting);
}

uint8_t MPU6050::getPowerCtl_1(void){
    
    return (int)SingleByteRead(PWR_MGMT_1_REG);
}

void MPU6050::setPowerCtl_2(uint8_t Conf){
    
    SingleByteWrite(PWR_MGMT_2_REG, Conf);
}

uint8_t MPU6050::getPowerCtl_2(void){
    
    return (int)SingleByteRead(PWR_MGMT_2_REG);
}

uint16_t MPU6050::getFIFOCount(void){
    
    uint16_t FIFOCount_HL, FIFOCount_L, FIFOCount_H;
    
    FIFOCount_L = SingleByteRead(FIFO_COUNTL_REG);
    FIFOCount_H = SingleByteRead(FIFO_COUNTH_REG);
    
    FIFOCount_HL = FIFOCount_H << 8 | FIFOCount_L;
    
    return FIFOCount_HL;
}

void MPU6050::FIFODataWrite(uint8_t Data){
    
    SingleByteWrite(FIFO_R_W_REG, Data);
}

uint8_t MPU6050::FIFODataRead(void){
    
    return (int)SingleByteRead(FIFO_R_W_REG);
}

uint8_t MPU6050::getWhoAmI(void){

    //WhoAmI Register address.
    return SingleByteRead(WHO_AM_I_REG);
}

char MPU6050::SingleByteRead(char address){
  
    /****info on the i2c_.read***
    address   : 8-bit I2C slave address [ addr | 1 ] 
    data      : Pointer to the byte-array to read data into 
    length    : Number of bytes to read 
    repeated  : Repeated start, true - don't send stop at end
    returns   : 0 on success (ack), or non-0 on failure (nack)
    */
  
   char tx = address;   //Address of register being accessed
   char output;         //Data read from the register
   
   i2c_.write((MPU6050_I2C_ADDRESS << 1) & 0xFE, &tx, 1);       //Access register indicated by the address
   i2c_.read((MPU6050_I2C_ADDRESS << 1) | 0x01, &output, 1);    //Read data from the register indicated by address and store it in output variable
   
   //i2c_.write(MPU6050_I2C_ADDRESS, &tx, 1);       //Access register indicated by the address
   //i2c_.read(MPU6050_I2C_ADDRESS, &output, 1);    //Read data from the register indicated by address and store it in output variable
    
    return output;
}

uint8_t MPU6050::SingleByteWrite(char address, char data){
    
    /****info on the i2c_.write***
    address   : 8-bit I2C slave address [ addr | 0 ]
    data      : Pointer to the byte-array data to send
    length    : Number of bytes to send
    repeated  : Repeated start, true - do not send stop at end
    returns   : 0 on success (ack), or non-0 on failure (nack)
    */
    
    int ack = 0;
    char tx[2]; //Two bytes to send during write operation, address and data
    
    tx[0] = address; //Address of register being accessed
    tx[1] = data;    //data to write to the register
    
    return   ack | i2c_.write((MPU6050_I2C_ADDRESS << 1) & 0xFE, tx, 2);   //Bitwise OR
}

void MPU6050::multiByteRead(char address, char* output, int size) {
    
    i2c_.write((MPU6050_I2C_ADDRESS << 1) & 0xFE, &address, 1);   //Access register indicated by the address
    i2c_.read((MPU6050_I2C_ADDRESS << 1) | 0x01, output, size);   //Read data from the register indicated by address and store it in output variable
}

uint8_t MPU6050::multiByteWrite(char address, char* ptr_data, int size) {
   
   int ack;
   ack = i2c_.write((MPU6050_I2C_ADDRESS << 1) & 0xFE, &address, 1);  //Access register indicated by the address
   
   return ack | i2c_.write((MPU6050_I2C_ADDRESS << 1) & 0xFE, ptr_data, size);   //Write data to the register indicated by address
                                    
}