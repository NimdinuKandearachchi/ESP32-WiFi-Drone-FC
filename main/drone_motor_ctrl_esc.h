#pragma once

#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define MOTOR_PWM_FRQ 500
#define MOTOR_PWM_RES LEDC_TIMER_12_BIT
#define MIN_MOTOR_SPEED 2048

class DRONE_MOTOR_CTRL{
    gpio_num_t mot[4];

public:
    int16_t mot_spd[4];
    int16_t max_mot[4];
    int16_t calib_mot[4];
    bool data_chg = false;

    DRONE_MOTOR_CTRL(gpio_num_t _m1, gpio_num_t _m2, gpio_num_t _m3, gpio_num_t _m4){
        mot[0] = _m1;
        mot[1] = _m2;
        mot[2] = _m3;
        mot[3] = _m4;
    }

    void init(){
        gpio_config_t mot_p_cfg = {
            .pin_bit_mask = (1ULL << this->mot[0]) | (1ULL << this->mot[1]) | (1ULL << this->mot[2]) | (1ULL << this->mot[3]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&mot_p_cfg));

        ledc_timer_config_t motor_timer_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = MOTOR_PWM_RES,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = MOTOR_PWM_FRQ,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&motor_timer_cfg));

        ledc_channel_config_t ledc_ch_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        for(uint8_t i = 0; i < 4; i++){
            ledc_ch_cfg.gpio_num = this->mot[i];
            ledc_ch_cfg.channel = (ledc_channel_t)i;
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch_cfg));
        }
        this->data_reset();
    }
    void data_reset(){
        for(uint8_t i = 0; i < 4; i++){
            this->max_mot[i] = 0;
            this->calib_mot[i] = 0;
        }
    }
    void break_motors(){
        this->set_motors(0,0,0,0);
        for(uint8_t i = 0; i < 4; i++)
            this->mot_spd[i] = 0;
    }

    void set_motors(){
        for(uint8_t i = 0; i < 4; i++){
            this->mot_spd[i] = (this->mot_spd[i] >= this->max_mot[i]) ? this->max_mot[i] : (this->mot_spd[i] + this->calib_mot[i]);
            this->mot_spd[i] = (this->mot_spd[i] < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : this->mot_spd[i];
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, this->mot_spd[i]);
        }

        for(uint8_t i = 0; i < 4; i++)
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }

    void set_motors(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4){
        m1 = (m1 >= this->max_mot[0]) ? this->max_mot[0] : (m1 < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : m1;
        m2 = (m2 >= this->max_mot[1]) ? this->max_mot[1] : (m2 < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : m2;
        m3 = (m3 >= this->max_mot[2]) ? this->max_mot[2] : (m3 < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : m3;
        m4 = (m4 >= this->max_mot[3]) ? this->max_mot[3] : (m4 < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : m4;

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, m1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, m2);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, m3);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, m4);

        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    }

};