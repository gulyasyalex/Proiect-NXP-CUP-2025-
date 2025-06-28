#ifndef _STEERINGWHEEL_H_
#define _STEERINGWHEEL_H_

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

#if ENABLE_ARDUINO == 1
  #include <Servo.h>
  class SteeringWheel : public Servo
#else
  #include <PWMServo.h>
  class SteeringWheel : public PWMServo
#endif

{
private:
    int ServoMaxLeftAngle;                
    int ServoMiddleAngle;               
    int ServoMaxRightAngle;
    
    float SteeringWheelMaxLeftRange;     // |120-90| = 30
    float SteeringWheelMaxRightRange;    // |30-90| = 60

    float SteeringWheelAngle;            // Current steering wheel angle
    float MappedSteeringWheelAngleLeft;  // Mapping factor for left span 
    float MappedSteeringWheelAngleRight; // Mapping factor for right span

public:
    //                                              120                                    90                                       30
    #if ENABLE_ARDUINO == 1
    SteeringWheel(unsigned int servo_max_left_angle = 120, unsigned int servo_middle_angle = 90, unsigned int servo_max_right_angle = 30): Servo(){
    #else
    SteeringWheel(unsigned int servo_max_left_angle = 120, unsigned int servo_middle_angle = 90, unsigned int servo_max_right_angle = 30): PWMServo(){
    #endif
    
        this->SteeringWheelAngle = 0;

        // Assign servo angle limits
        this->ServoMaxLeftAngle = (int)servo_max_left_angle;
        this->ServoMiddleAngle = (int)servo_middle_angle;
        this->ServoMaxRightAngle = (int)servo_max_right_angle;

        // Assign Range
        this->SteeringWheelMaxLeftRange = abs((float)(this->ServoMaxLeftAngle - this->ServoMiddleAngle));
        this->SteeringWheelMaxRightRange = abs((float)(this->ServoMaxRightAngle - this->ServoMiddleAngle));

        // Calculate servo angle span based on left angle it gives ratio 1:2
        // For each 1 angle to the left, you need 2 to the right
        this->MappedSteeringWheelAngleLeft = SteeringWheelMaxLeftRange / SteeringWheelMaxLeftRange;
        this->MappedSteeringWheelAngleRight = SteeringWheelMaxRightRange / SteeringWheelMaxLeftRange;

        double MaxSteering = MIN(this->SteeringWheelMaxLeftRange, this->SteeringWheelMaxRightRange);
        this->SteeringWheelMaxLeftRange = -MaxSteering;
        this->SteeringWheelMaxRightRange = MaxSteering;
    }

    ~SteeringWheel() {}

    // Sets the steering angle in degrees and maps it to the servo angle
    // maxLeft:30 ; maxRight:-30
    void setSteeringAngleDeg(float steering_angle)
    {
        int new_servo_angle = this->ServoMiddleAngle;

        if (steering_angle > 0) // Going right
        {
            /*
                90 middle 
                steering_angle(ex. 10) * MappedSteeringWheelAngleRight(2) = 20 =>  90 - 20 = 70 
                SERVO written value is 70
            */
            steering_angle = MIN(steering_angle, this->SteeringWheelMaxRightRange); // Clamp to max right angle
            this->SteeringWheelAngle = steering_angle;
            new_servo_angle = this->ServoMiddleAngle - (int)(steering_angle * this->MappedSteeringWheelAngleRight);
        }
        else if (steering_angle < 0) // Going left
        {
            /*
                90 middle 
                steering_angle(ex. -10) * MappedSteeringWheelAngleRight(1) = -10 =>  90 - (-10) = 100 
                SERVO written value is 70
            */            
            steering_angle = MAX(steering_angle, this->SteeringWheelMaxLeftRange); // Clamp to max left angle
            this->SteeringWheelAngle = steering_angle;
            new_servo_angle = this->ServoMiddleAngle - (int)(steering_angle * this->MappedSteeringWheelAngleLeft);
        }
        else // Going middle
        {
            this->SteeringWheelAngle = 0;
            new_servo_angle = this->ServoMiddleAngle;
        }

        this->write(new_servo_angle);
    }

    // Get the current steering angle
    float getSteeringAngle()
    {
        return this->SteeringWheelAngle;
    }

    // Get the maximum left steering wheel angle
    float getSteeringWheelMaxLeftRange()
    {
        return this->SteeringWheelMaxLeftRange;
    }

    // Get the maximum right steering wheel angle
    float getSteeringWheelMaxRightRange()
    {
        return this->SteeringWheelMaxRightRange;
    }

    // Get the mapping factor for left span
    float getMappedSteeringWheelAngleLeft()
    {
        return this->MappedSteeringWheelAngleLeft;
    }

    // Get the mapping factor for right span
    float getMappedSteeringWheelAngleRight()
    {
        return this->MappedSteeringWheelAngleRight;
    }
};
#endif