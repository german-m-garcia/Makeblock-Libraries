/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeGyroRaw
 * \brief   Modified version of Makeblock's Driver for MeGyro module following https://github.com/kriswiner/MPU6050
 * @file    MeGyroRaw.cpp
 * @author  MakeBlock
 * @version V1.0.5
 * @date    2021
 * @brief   Modified version of Makeblock's Driver for MeGyro module.
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for MeGyro module, It supports MeGyro V1.0 device provided
 * by MakeBlock.
  
 */

/* Includes ------------------------------------------------------------------*/
#include "MeGyroRaw.h"

/* Private functions ---------------------------------------------------------*/
#ifdef ME_PORT_DEFINED

MeGyroRaw::MeGyroRaw(void) : MePort(0)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
}

MeGyroRaw::MeGyroRaw(uint8_t port) : MePort(port)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
}

MeGyroRaw::MeGyroRaw(uint8_t port, uint8_t address) : MePort(port)
{
  Device_Address = address;
}
#else  // ME_PORT_DEFINED

MeGyroRaw::MeGyroRaw(uint8_t AD0, uint8_t INT)
{
  Device_Address = GYRO_DEFAULT_ADDRESS;
  _AD0 = AD0;
  _INT = INT;
}

MeGyroRaw::MeGyroRaw(uint8_t AD0, uint8_t INT, uint8_t address)
{
  Device_Address = address;
  _AD0 = AD0;
  _INT = INT;
}
#endif // ME_PORT_DEFINED

void MeGyroRaw::setpin(uint8_t AD0, uint8_t INT)
{
  _AD0 = AD0;
  _INT = INT;
#ifdef ME_PORT_DEFINED
  s1 = AD0;
  s2 = INT;
#endif // ME_PORT_DEFINED
}

void MeGyroRaw::begin(void)
{
  // gSensitivity = 65.5; //for 500 deg/s, check data sheet
  gSensitivity = getGsensitivity(gscale);

  accRes = getAres(ascale);
  gyroRes = getGres(gscale);

  Wire.begin();
  delay(200);
  writeReg(0x6b, 0x00); //close the sleep mode
  delay(100);
  writeReg(0x1a, 0x01); //configurate the digital low pass filter
  delay(100);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(Device_Address, GYRO_CONFIG);
  writeByte(Device_Address, GYRO_CONFIG, c & ~0xE0);       // Clear self-test bits [7:5]
  writeByte(Device_Address, GYRO_CONFIG, c & ~0x18);       // Clear AFS bits [4:3]
  writeByte(Device_Address, GYRO_CONFIG, c | gscale << 3); // Set full scale range for the gyro

  // Set accelerometer configuration
  c = readByte(Device_Address, ACCEL_CONFIG);
  writeByte(Device_Address, ACCEL_CONFIG, c & ~0xE0);       // Clear self-test bits [7:5]
  writeByte(Device_Address, ACCEL_CONFIG, c & ~0x18);       // Clear AFS bits [4:3]
  writeByte(Device_Address, ACCEL_CONFIG, c | ascale << 3); // Set full scale range for the accelerometer

  calibrateMPU6050(gyroBias, accelBias);
}

void MeGyroRaw::update(void)
{

  int8_t return_value;

  /* read acc data */
  return_value = readData(0x3b, i2cData, 6);
  if (return_value != 0)
  {
    return;
  }
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  gForceX = accX * accRes - accelBias[0];
  gForceY = accY * accRes - accelBias[1];
  gForceZ = accZ * accRes - accelBias[2];

  /* read rotation data */
  return_value = readData(0x43, i2cData, 6);
  if (return_value != 0)
  {
    return;
  }

  gyrX = ((i2cData[0] << 8) | i2cData[1]);
  gyrY = ((i2cData[2] << 8) | i2cData[3]);
  gyrZ = ((i2cData[4] << 8) | i2cData[5]);

  rotX = gyrX / gSensitivity - gyroBias[0];
  rotY = gyrY / gSensitivity - gyroBias[1];
  rotZ = gyrZ / gSensitivity - gyroBias[2];
}

uint8_t MeGyroRaw::getDevAddr(void)
{
  return Device_Address;
}

double MeGyroRaw::getAngleX(void)
{
  return rotX;
}

double MeGyroRaw::getAngleY(void)
{
  return rotY;
}

double MeGyroRaw::getAngleZ(void)
{
  return rotZ;
}

double MeGyroRaw::getAccX(void)
{
  return gForceX;
}

double MeGyroRaw::getAccY(void)
{
  return gForceY;
}

double MeGyroRaw::getAccZ(void)
{
  return gForceZ;
}

double MeGyroRaw::getAngle(uint8_t index)
{
  if (index == 1)
  {
    return getAngleX();
  }
  else if (index == 2)
  {
    return getAngleY();
  }
  else if (index == 3)
  {
    return getAngleZ();
  }
}

double MeGyroRaw::getGsensitivity(int gscale)
{
  if (gscale == GFS_250DPS)
    return 131.0;

  if (gscale == GFS_500DPS)
    return 65.5;

  if (gscale == GFS_1000DPS)
    return 32.8;

  if (gscale == GFS_2000DPS)
    return 16.4;

  return -1.0;
}

double MeGyroRaw::getGres(int gscale)
{

  if (gscale == GFS_250DPS)
    return 250.0 / 32768.0;

  if (gscale == GFS_500DPS)
    return 500.0 / 32768.0;

  if (gscale == GFS_1000DPS)
    return 1000.0 / 32768.0;

  if (gscale == GFS_2000DPS)
    return 2000.0 / 32768.0;

  return -1.0;
}

double MeGyroRaw::getAres(int ascale)
{
  if (ascale == AFS_2G)
    return 2.0 / 32768.0;

  if (ascale == AFS_4G)
    return 4.0 / 32768.0;

  if (ascale == AFS_8G)
    return 8.0 / 32768.0;

  if (ascale == AFS_4G)
    return 16.0 / 32768.0;

  return -1.0;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MeGyroRaw::calibrateMPU6050(double *dest1, double *dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(Device_Address, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(Device_Address, PWR_MGMT_1, 0x01);
  writeByte(Device_Address, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(Device_Address, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(Device_Address, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(Device_Address, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(Device_Address, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(Device_Address, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(Device_Address, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(Device_Address, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  writeByte(Device_Address, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
  writeByte(Device_Address, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(Device_Address, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
  uint16_t accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(Device_Address, USER_CTRL, 0x40); // Enable FIFO
  writeByte(Device_Address, FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80);                                  // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(Device_Address, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
  readBytes(Device_Address, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(Device_Address, FIFO_R_W, 12, &data[0]);            // read data for averaging
    accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t)accel_temp[1];
    accel_bias[2] += (int32_t)accel_temp[2];
    gyro_bias[0] += (int32_t)gyro_temp[0];
    gyro_bias[1] += (int32_t)gyro_temp[1];
    gyro_bias[2] += (int32_t)gyro_temp[2];
  }
  accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t)packet_count;
  accel_bias[2] /= (int32_t)packet_count;
  gyro_bias[0] /= (int32_t)packet_count;
  gyro_bias[1] /= (int32_t)packet_count;
  gyro_bias[2] /= (int32_t)packet_count;

  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t)accelsensitivity;
  } // Remove gravity from the z-axis accelerometer bias calculation
  else
  {
    accel_bias[2] += (int32_t)accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

  dest1[0] = (double)gyro_bias[0] / (double)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (double)gyro_bias[1] / (double)gyrosensitivity;
  dest1[2] = (double)gyro_bias[2] / (double)gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0};               // A place to hold the factory accelerometer trim biases
  readBytes(Device_Address, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(Device_Address, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(Device_Address, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++)
  {
    if (accel_bias_reg[ii] & mask)
      mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (double)accel_bias[0] / (double)accelsensitivity;
  dest2[1] = (double)accel_bias[1] / (double)accelsensitivity;
  dest2[2] = (double)accel_bias[2] / (double)accelsensitivity;
}

void MeGyroRaw::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.write(data);                // Put data in Tx buffer
  Wire.endTransmission();          // Send the Tx buffer
}

uint8_t MeGyroRaw::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                          // data will store the register data
  Wire.beginTransmission(address);       // Initialize the Tx buffer
  Wire.write(subAddress);                // Put slave register address in Tx buffer
  Wire.endTransmission(false);           // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t)1); // Read one byte from slave register address
  data = Wire.read();                    // Fill Rx buffer with result
  return data;                           // Return data read from slave register
}

void MeGyroRaw::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress);          // Put slave register address in Tx buffer
  Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count); // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  } // Put read results in the Rx buffer
}

int8_t MeGyroRaw::writeReg(int16_t reg, uint8_t data)
{
  int8_t return_value = 0;
  return_value = writeData(reg, &data, 1);
  return (return_value);
}

int8_t MeGyroRaw::readData(uint8_t start, uint8_t *buffer, uint8_t size)
{
  int16_t i = 0;
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if (return_value != 1)
  {
    return (I2C_ERROR);
  }
  return_value = Wire.endTransmission(false);
  if (return_value != 0)
  {
    return (return_value);
  }
  delayMicroseconds(1);
  /* Third parameter is true: relase I2C-bus after data is read. */
  Wire.requestFrom(Device_Address, size, (uint8_t) true);
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  delayMicroseconds(1);
  if (i != size)
  {
    return (I2C_ERROR);
  }
  return (0); //return: no error
}

int8_t MeGyroRaw::writeData(uint8_t start, const uint8_t *pData, uint8_t size)
{
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if (return_value != 1)
  {
    return (I2C_ERROR);
  }
  Wire.write(pData, size);
  return_value = Wire.endTransmission(true);
  return (return_value); //return: no error
}
