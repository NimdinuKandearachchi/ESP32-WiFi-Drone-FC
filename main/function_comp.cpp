extern "C" {
    #include "stdint.h"
    #include "stdio.h"
    #include "string.h"

    #include "esp_log.h"
    #include "esp_timer.h"
}

#define PID_CTRL_I_TERM_MAX 400

struct LOW_PASS_FILTER {
    float prv_value = 0;
    float coeff = 0.6f;
    float get_filtered(float value){
        value = this->coeff * this->prv_value + (1 - this->coeff) * value;
        this->prv_value = value;
        return value;
    }
};

/// @brief uses ESP32 libraries !!
class PID_Controller {
private:
    float Kpid[3] = {0};
    float term_I = 0;
    float prv_error = 0;
    int64_t prv_time = 0;

    LOW_PASS_FILTER D_lf;

public:
    PID_Controller(){
        this->D_lf.coeff = 0.6f;
    }
    
    void* get_kpid_ptr(){
        return &this->Kpid;
    }
    void reset_term_I(){
        this->term_I = 0;
    }
    float calculate_pid(float input, float desired){
        float dt = this->get_dt();

        float error = desired - input;

        float D = (error - prv_error)/dt;
        this->prv_error = error;
        D = this->D_lf.get_filtered(D);

        this->term_I += error*dt;
        this->term_I = (this->term_I > PID_CTRL_I_TERM_MAX) ? PID_CTRL_I_TERM_MAX : 
                        (this->term_I < -PID_CTRL_I_TERM_MAX) ? -PID_CTRL_I_TERM_MAX : this->term_I;
        
        float pid_value = error*this->Kpid[0] + this->term_I*this->Kpid[1] + D*this->Kpid[2];
        return pid_value;
    }
    
    float get_dt(){
        float dt = (esp_timer_get_time() - (int64_t)this->prv_time) / 1e6f;
        this->prv_time = esp_timer_get_time();
        return dt;
    }
};
