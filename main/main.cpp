
extern "C" {
    #include <stdio.h>
    #include <stdint.h>
    #include "string.h"

    #include "esp_log.h"
    #include "esp_system.h"
    #include "esp_timer.h"
    #include "driver/gpio.h"

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/timers.h"
}
// GY_INT = 2 | Pixel_led - 5 | rx2 = 16 | tx2 = 17 | buz = 23
// m1 = 18 | m2 = 19 | m3 = 21 | m4 = 22 | 12v_ref = sen_vp
// scl = 4 | sda = 15
// hspi_miso = 12 | hspi_mosi = 13 | hspi_sck = 14
#define ESP32_MODEL_NAME C3 // 0 - esp32 , C3 - esp32 c3
#define BUZZ_PIN GPIO_NUM_8

void ws_ctrl_data(uint8_t *data);
void buzz_beep(uint16_t tms);
uint8_t running_task = 0;

#include "global_fun.h"
#include "drone_motor_ctrl_esc.h"
#include "nvs_data_store.h"
#include "wifi_ws_handler.h"
#include "i2c_driver_class.h"
#include "mpu6500_handler.cpp"

TaskHandle_t drone_motor_task_h;

NVS_DATA_STORE nvs_f;
ESP_WIFI_DEV WiFi_drv(&nvs_f);

#if ESP32_MODEL_NAME == C3
    DRONE_MOTOR_CTRL dmotor(GPIO_NUM_7, GPIO_NUM_5, GPIO_NUM_2, GPIO_NUM_10);
    ESP_I2C_IDF i2c_driver(3,4);
#elif ESP32_MODEL_NAME == 0
    DRONE_MOTOR_CTRL dmotor(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22);
    ESP_I2C_IDF i2c_driver(15,4);
#else
    #error "ESP32 model not specified"
#endif

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))
    uint8_t deb_info_num = 0;

int16_t drone_ati = 0, drone_roll = 0, drone_pitch = 0;

void ws_ctrl_data(uint8_t *data){
    drone_roll = data[1] << 8 | data[0];
    drone_ati = data[3] << 8 | data[2];
    drone_pitch = data[5] << 8 | data[4];

    //////
    // printf("ws rx dt: ati: %d, roll: %d, pitch: %d\n", drone_ati, drone_roll,drone_pitch);
    // if(deb_info_num == 3) WiFi_drv.ws_dt_Str_send("DEB","ws rx dt: ati: %d, roll: %d, pitch: %d\n", drone_ati, drone_roll,drone_pitch);
}

float mot_Kpid[3];
float mot_Kpid_yaw[3];
float pre_ang_err[3] = {0};
float pid_ang_I[3] = {0};
float pid_prv_D[3] = {0};
uint64_t pid_pre_time[2] = {0};

    uint8_t printin_tiII = 0;

int16_t get_motor_pid(float input, float desired, uint8_t angle_type){
    float dt = (float)(esp_timer_get_time() - pid_pre_time[angle_type])/1000000;
    pid_pre_time[angle_type] = esp_timer_get_time();

    float error = 0,D = 0;
    error = desired - input;

    pid_ang_I[angle_type] += error * dt;
    // pid_ang_I[angle_type] += (float)(error);
    pid_ang_I[angle_type] = (pid_ang_I[angle_type] > 400) ? 400 : (pid_ang_I[angle_type] < -400) ? -400 : pid_ang_I[angle_type];

    D = (error - pre_ang_err[angle_type]) / dt;
    D = 0.6 * pid_prv_D[angle_type] + (1 - 0.6) * D;
    pid_prv_D[angle_type] = D;
    // D = (float)(error - pre_ang_err[angle_type]);
    pre_ang_err[angle_type] = error;

    float pid = error*mot_Kpid[0] + pid_ang_I[angle_type]*mot_Kpid[1] + D*mot_Kpid[2];

    // printin_tiII++;
    //     if(printin_tiII >= 20) {
    //         if(deb_info_num == 1) WiFi_drv.ws_dt_Str_send("DEB", "error:%f pid_ang_I:%f D:%f pid:%f\n", error, pid_ang_I[angle_type], D, pid);
    //         if(deb_info_num == 2) WiFi_drv.ws_dt_Str_send("DEB", "pid = %f + %f + %f = %f\n", (float)error*mot_Kpid[0], (float)pid_ang_I[angle_type]*mot_Kpid[1], (float)D*mot_Kpid[2], pid);
    //         printin_tiII = 0;
    //     }

    return pid;
}

int16_t get_motor_yaw_pid(float input, float desired = 0){
    float error = desired - input;
    float D = 0;

    pid_ang_I[2] += error;
    pid_ang_I[2] = (pid_ang_I[2] > 400) ? 400 : (pid_ang_I[2] < -400) ? -400 : pid_ang_I[2];
    
    D = (error - pre_ang_err[2]);
    D = 0.7 * pid_prv_D[2] + (1 - 0.7) * D;
    pid_prv_D[2] = D;
    pre_ang_err[2] = error;

    float pid = error*mot_Kpid_yaw[0] + pid_ang_I[2]*mot_Kpid_yaw[1] + D*mot_Kpid_yaw[2];

    return pid;
}

#define MAX_ROLL_PITCH_ANG 10
#define STICK_DEADZONE 5
#define INPUT_LPF_FACTOR 0.7

int16_t roll_pid = 0, pitch_pid = 0, yaw_pid = 0;
float desired_roll = 0, desired_pitch = 0;
float prvdesired_roll = 0, prvdesired_pitch = 0;

void drone_mot_cntrol(){
    desired_roll = (drone_roll > STICK_DEADZONE || drone_roll < -STICK_DEADZONE) ? (float)drone_roll * MAX_ROLL_PITCH_ANG / 150 : 0;
    desired_pitch = (drone_pitch > STICK_DEADZONE || drone_pitch < -STICK_DEADZONE) ? (float)drone_pitch * MAX_ROLL_PITCH_ANG / 150 : 0;

    desired_roll = INPUT_LPF_FACTOR * prvdesired_roll + (1 - INPUT_LPF_FACTOR) * desired_roll;
    desired_pitch = INPUT_LPF_FACTOR * prvdesired_pitch + (1 - INPUT_LPF_FACTOR) * desired_pitch;
    prvdesired_roll = desired_roll;
    prvdesired_pitch = desired_pitch;

    if(drone_ati > 2050){
        if(mpuIMU_roll > 0.3 || mpuIMU_roll < -0.3) roll_pid = get_motor_pid(mpuIMU_roll, desired_roll, 0);
        if(mpuIMU_pitch > 0.3 || mpuIMU_pitch < -0.3) pitch_pid = get_motor_pid(mpuIMU_pitch, desired_pitch, 1);
        if(mpuIMU_yaw > 1 || mpuIMU_yaw < -1) yaw_pid = get_motor_yaw_pid(mpuIMU_yaw, 0);

        dmotor.mot_spd[0] = 1.05*(drone_ati + roll_pid + pitch_pid + yaw_pid);
        dmotor.mot_spd[1] = 1.05*(drone_ati + roll_pid - pitch_pid - yaw_pid);
        dmotor.mot_spd[2] = 1.05*(drone_ati - roll_pid - pitch_pid + yaw_pid);
        dmotor.mot_spd[3] = 1.05*(drone_ati - roll_pid + pitch_pid - yaw_pid);
    }
    else{
        dmotor.mot_spd[0] = 0;
        dmotor.mot_spd[1] = 0;
        dmotor.mot_spd[2] = 0;
        dmotor.mot_spd[3] = 0;
        pid_ang_I[0] = 0;
        pid_ang_I[1] = 0;
        pid_ang_I[2] = 0;
    }

    dmotor.data_chg = true;
}

void drone_motor_task(void* arg){
    TickType_t xLastWakeTime; 
    const TickType_t xFrequency = 5;
    xLastWakeTime = xTaskGetTickCount ();

    // uint8_t printin_ti = 0;
    // uint64_t looptime = 0;

    // uint16_t fps = 0;
    // uint64_t fpstime = 0;

    while (1){
        // looptime = esp_timer_get_time();

        if(mpu6050_task()){
            drone_mot_cntrol();
        
            // printin_ti++;
            // if(printin_ti >= 20) {

            // //     printf(">m1:%d,m2:%d,m3:%d,m4:%d,ati:%d \n",
            // //                 dmotor.mot_spd[0],dmotor.mot_spd[1],dmotor.mot_spd[2],dmotor.mot_spd[3],drone_ati);
            //     printin_ti = 0;
            //     WiFi_drv.ws_dt_Str_send("DEB", "roll:%f pitch=%f yaw=%f\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
            //     // printf(">roll:%f,pitch=%f,yaw=%f\r\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
            // }

            dmotor.set_motors();
            
                // printf(">roll:%f,pitch:%f,yaw:%f\r\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
            // fps++;

            // if(esp_timer_get_time() >= fpstime){
            //     fpstime = esp_timer_get_time() + 1000000;
            //     printf("#>>>>>>>>>>>>>>>>>>> mpu fps : %d\n",fps);
            //     fps = 0;
            // }

            // looptime = (float)(2000 + looptime - esp_timer_get_time()) / 1000;
            // vTaskDelay(pdMS_TO_TICKS(looptime));

            xTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // float motor_corr_vals[3];
        // memset(motor_corr_vals, 0, sizeof(motor_corr_vals));
        // if(drone_ati > 0) mpu6.get_motor_inputs(motor_corr_vals, drone_roll, drone_pitch);
        // mpu6.debug_angles();

        // dmotor.mot_spd[0] = drone_ati - motor_corr_vals[0] - motor_corr_vals[1] - motor_corr_vals[2];
        // dmotor.mot_spd[1] = drone_ati - motor_corr_vals[0] + motor_corr_vals[1] + motor_corr_vals[2];
        // dmotor.mot_spd[2] = drone_ati + motor_corr_vals[0] + motor_corr_vals[1] - motor_corr_vals[2];
        // dmotor.mot_spd[3] = drone_ati + motor_corr_vals[0] - motor_corr_vals[1] + motor_corr_vals[2];

        

        // looptime = (float)(4000 + looptime - esp_timer_get_time()) / 1000;
        // vTaskDelay(pdMS_TO_TICKS(looptime));
    //     vTaskDelay(pdMS_TO_TICKS(1));
    }
    

}
// TimerHandle_t ESP_MOTOR_TASK_H;
void stop_drone_motor_task(){
    if(drone_motor_task_h != NULL) vTaskSuspend(drone_motor_task_h);
    // if(ESP_MOTOR_TASK_H != NULL) xTimerStop(ESP_MOTOR_TASK_H, 0);
    // mpu6.reset_pid();
}
// -1114 1330 790 | 2 74 -68
int16_t mpu6050OffsetData[6] = {-1112, 1330, 792, 2, 73, -68};

void drone_init(){

    dmotor.init();
    dmotor.break_motors();

    buz_pin_init(BUZZ_PIN);

    nvs_f.init();
    nvs_f.reg_var_nvs("motm", &dmotor.max_mot, sizeof(dmotor.max_mot));
    nvs_f.reg_var_nvs("motc", &dmotor.calib_mot, sizeof(dmotor.calib_mot));
    nvs_f.reg_var_nvs("kpid", mot_Kpid, sizeof(mot_Kpid));
    nvs_f.reg_var_nvs("kryw", mot_Kpid_yaw, sizeof(mot_Kpid_yaw));
    // nvs_f.reg_var_nvs("mpuc", mpu6050OffsetData, sizeof(mpu6050OffsetData));
    
    // nvs_f.reg_var_nvs("kxcl", &initial_roll, sizeof(initial_roll));
    // nvs_f.reg_var_nvs("kycl", &initial_pitch, sizeof(initial_pitch));
    // nvs_f.reg_var_nvs("kzcl", &initial_yaw, sizeof(initial_yaw));

    nvs_f.recoverData();

    WiFi_drv.init();

    i2c_driver.install_i2c_driver();
    // mpu6.begin();
    mpu6050_init_task(&buzz_beep, mpu6050OffsetData);

    vTaskDelay(pdMS_TO_TICKS(100));

    xTaskCreatePinnedToCore(&drone_motor_task, "drone_motor_task", 1024*8, NULL, 20, &drone_motor_task_h, 0);
    vTaskSuspend(drone_motor_task_h);
}

void indicatorMotor(){
    dmotor.set_motors(MIN_MOTOR_SPEED+150, 0, 0, 0);
    vTaskDelay(500);
    dmotor.set_motors(0, MIN_MOTOR_SPEED+150, 0, 0);
    vTaskDelay(500);
    dmotor.set_motors(0, 0, MIN_MOTOR_SPEED+150, 0);
    vTaskDelay(500);
    dmotor.set_motors(0, 0, 0, MIN_MOTOR_SPEED+150);
    vTaskDelay(500);
    dmotor.break_motors();
    vTaskDelay(500);
}

extern "C" void app_main(void){
    printf("BOOTING...\n");
    printf("SETUP IN PROG...\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    drone_init();

    uint8_t pre_running_task = running_task;

    printf("done\n");
    buzz_beep(1000);
    indicatorMotor();

    // uint64_t ws_fps_nxt_up = 0;

    while (1) {
        if(running_task != pre_running_task){
            pre_running_task = running_task;
            buzz_beep(100);
        }
        // char serial_data[16];
        // fgets(serial_data, sizeof(serial_data), stdin);

        // if(esp_timer_get_time() >= ws_fps_nxt_up){
        //     ws_fps_nxt_up = esp_timer_get_time() + 1000000;
        //     WiFi_drv.ws_dt_Str_send("DEB", "fps m: %d g: %d\n", motor_fps, gyro_fps);
        //     motor_fps = 0;
        //     gyro_fps = 0;
        // }
        
        switch(running_task){
            case 0: // stop
                stop_drone_motor_task();
                dmotor.break_motors();
                running_task = 1;
                break;

            case 1: // idle
                vTaskDelay(pdMS_TO_TICKS(20));
                break;

            case 2: // run
                vTaskResume(drone_motor_task_h);
                // if(ESP_MOTOR_TASK_H != NULL) xTimerStart(ESP_MOTOR_TASK_H, 0);
                // drone_motor_run = true;
                running_task = 1;
                // drone_mot_cntrol();
                // if(dmotor.data_chg){
                // dmotor.set_motors();
                    // printf("motors: m1[%d] m2[%d] m3[%d] m4[%d] \n",
                    //     dmotor.mot_spd[0],dmotor.mot_spd[1],dmotor.mot_spd[2],dmotor.mot_spd[3]);
                // dmotor.data_chg = false;
                // }
                // vTaskDelay(pdMS_TO_TICKS(10));

                break;

            case 3: // calibrate
                printf( "calibration starting...\n");
                WiFi_drv.ws_dt_Str_send("DEB", "calibration starting...\n");

                stop_drone_motor_task();
                dmotor.break_motors();

                vTaskDelay(pdMS_TO_TICKS(500));

                mpu6050_calibrate(mpu6050OffsetData);
                WiFi_drv.ws_dt_Str_send("DEB", "calibrated saving data...\n");

                vTaskDelay(pdMS_TO_TICKS(100));
                nvs_f.writeToNVS("mpuc");
                buzz_beep(100);
                vTaskDelay(pdMS_TO_TICKS(100));

                WiFi_drv.ws_dt_Str_send("DEB", "calibration completed...\n");
                indicatorMotor();
                running_task = 6;
                break;
            case 4:
                // vTaskDelete(mpu6050_task_h);
                running_task = 0;
                break;
            case 5:
                i2c_driver.i2c_scan();
                running_task = 0;
                break;

            case 6: // get calibrate data
                printf("#### offsets  : %d %d %d | %d %d %d\n", mpu6050OffsetData[0], mpu6050OffsetData[1], mpu6050OffsetData[2], mpu6050OffsetData[3], mpu6050OffsetData[4], mpu6050OffsetData[5]);
                WiFi_drv.ws_dt_Str_send("DEB", "#### mpu6050OffsetData : %d %d %d | %d %d %d\n", mpu6050OffsetData[0], mpu6050OffsetData[1], mpu6050OffsetData[2], mpu6050OffsetData[3], mpu6050OffsetData[4], mpu6050OffsetData[5]);
                running_task = 0;
                break;

            case 7:
                WiFi_drv.ws_dt_Str_send("DEB", "kpid kp: %f ki: %f kd: %f\n", mot_Kpid[0], mot_Kpid[1], mot_Kpid[2]);
                
                running_task = 1;
                break;
            case 8:
                debug_output_type = 1;
                running_task = 1;
                break;
            case 9:
                debug_output_type = 2;
                running_task = 1;
                break;
            case 10:
                debug_output_type = 0;
                running_task = 1;
                break;
            case 11:
                deb_info_num = 1;
                running_task = 1;
                break;
            case 12:
                deb_info_num = 2;
                running_task = 1;
                break;
            case 13:
                deb_info_num = 0;
                running_task = 1;
                break;
            case 14:
                deb_info_num = 3;
                running_task = 1;
                break;
            
            

            default:
                running_task = 0;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}



// bool isRunningMpuTask = false;
// void stop_drone_motor_task(bool stop){
//     // if(drone_motor_task_h != NULL) vTaskSuspend(drone_motor_task_h);
//     // if(ESP_MOTOR_TASK_H != NULL) xTimerStop(ESP_MOTOR_TASK_H, 0);
//     // mpu6.reset_pid();

//     isRunningMpuTask = !stop;

//     if(stop)
//         gpio_intr_disable(GPIO_NUM_1);
//     else
//         gpio_intr_enable(GPIO_NUM_1);
// }

// void setupMpuInterrupt(){
//     gpio_config_t gCfg = {
//         .pin_bit_mask = (1ULL << GPIO_NUM_1),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_ENABLE,
//         .intr_type = GPIO_INTR_NEGEDGE,
//     };
//     gpio_config(&gCfg);

//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(GPIO_NUM_1, mpuInterruptTask, NULL);

//     ESP_LOGI(TAG, "GPIO Interrupt Configured on GPIO");

//     isRunningMpuTask = true;
// }

