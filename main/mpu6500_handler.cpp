#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_driver_class.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h"

#define RAD_TO_DEG 57.29578  //(180.0/M_PI)
#define DEG_TO_RAD 0.0174533

#define micros() (esp_timer_get_time())

const char* TAG = "MPU";

MPU6050 mpu;

float mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw; /// final vals

uint16_t mpuIMU_fifoCount;		// count of all bytes currently in FIFO
uint8_t mpuIMU_fifoBuf[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector


// display quaternion values in easy matrix form: w x y z
void getQuaternion() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	// printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
}

// display Euler angles in degrees
void getEuler() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetEuler(euler, &q);
	// printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}

// display Euler angles in degrees
void getYawPitchRoll() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	mpuIMU_roll = ypr[2] * RAD_TO_DEG;
	mpuIMU_pitch = ypr[1] * RAD_TO_DEG;
	mpuIMU_yaw = ypr[0] * RAD_TO_DEG;

	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	// ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	// printf(">roll:%f,pitch:%f,yaw:%f\r\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetAccel(&aa, mpuIMU_fifoBuf);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	// printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetAccel(&aa, mpuIMU_fifoBuf);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	// printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}

void mpu6050_calibrate(int16_t *offsets){
	
	mpu.setDMPEnabled(false);
	mpu.resetFIFO();
	vTaskDelay(pdMS_TO_TICKS(100));

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);
	vTaskDelay(pdMS_TO_TICKS(100));

	mpu.CalibrateAccel(6);
	vTaskDelay(pdMS_TO_TICKS(100));
	mpu.CalibrateGyro(6);
	vTaskDelay(pdMS_TO_TICKS(100));

	int16_t* offmp = mpu.GetActiveOffsets();
	memcpy(offsets, offmp, sizeof(int16_t)*6);

	mpu.setDMPEnabled(true);
}

void mpu6050_init_task(void buz(uint16_t gg), int16_t* offsets){
	mpu.initialize();

	// initilize dmp success = 0
	if(mpu.dmpInitialize() != 0){
		ESP_LOGE(TAG, "DMP initialization failed ...");
		while(1){
			buz(20);
			vTaskDelay(40);
		}
	}
	
	// mpu6050_calibrate(offsets);

	mpu.setXAccelOffset(offsets[0]);
	mpu.setYAccelOffset(offsets[1]);
	mpu.setZAccelOffset(offsets[2]);
	mpu.setXGyroOffset(offsets[3]);
	mpu.setYGyroOffset(offsets[4]);
	mpu.setZGyroOffset(offsets[5]);

	// mpu.setInterruptLatchClear(true);
	// mpu.setIntDMPEnabled(true);

	mpu.setDMPEnabled(true);

	mpuIMU_fifoCount = mpu.dmpGetFIFOPacketSize();
}

uint8_t printin_tig = 0;
uint8_t debug_output_type = 0;

bool mpu6050_task(){
	uint16_t fifobufsz = mpu.getFIFOCount();

	if(fifobufsz >= mpuIMU_fifoCount){
		if(!mpu.dmpGetCurrentFIFOPacket(mpuIMU_fifoBuf))return false;
		getYawPitchRoll();
		return true;
	}

	return false;


	// bool gotdata = false;
    // if (mpu.dmpGetCurrentFIFOPacket(mpuIMU_fifoBuf)) { // Get the Latest packet
	// 		getYawPitchRoll();
	// 		gotdata = true;
	// }
	// else{
	// 		gotdata = false;
	// }
		// printin_tig++;
        // if(printin_tig >= 20) {
			// printf(">accX: %f,accY:%f,accZ:%f,gyX:%f,gyY:%f,gyZ:%f\n", ax, ay, az, gx, gy, gz);
			// printf( ">roll:%f,pitch:%f,yaw:%f\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
       	 	// if(debug_output_type == 1) WiFi_drv.ws_dt_Str_send("DEB", "acc: %f %f %f - gy: %f %f %f\n", ax, ay, az, gx, gy, gz);
            // else if(debug_output_type == 2) WiFi_drv.ws_dt_Str_send("DEB", "roll:%f pitch=%f yaw=%f\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
   		// 	printin_tig = 0;
        // }

	// return gotdata;
}
