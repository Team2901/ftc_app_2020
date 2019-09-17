package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class SwerveTestTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = 24.0/80.0;
    public final static int SERVO_MAX_ANGLE = 180;
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
    public double wheelServoAngle(double wheelAngle){
        double servoAngle = WHEEL_SERVO_GEAR_RATIO * wheelAngle;
        return servoAngle;
    }

    public double servoAngleToPosition(double servoAngle){
       double servoPosition = servoAngle/SERVO_MAX_ANGLE;
       return servoPosition;
    }
}
