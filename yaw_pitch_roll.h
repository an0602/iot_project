#ifndef yaw_pitch_roll_h
#define yaw_pitch_roll_h

#include <Arduino.h>
#include <Adafruit_BNO08x.h>



class yaw_pitch_roll
{
    public:
        struct euler_t {
            float yaw;
            float pitch;
            float roll;
        } ypr;
        yaw_pitch_roll();
        void imuSetup(void);
        void imuGetYawPitchRoll();

    private:
        void setReports(sh2_SensorId_t reportType, long report_interval);
        void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
        void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees);

};

#endif