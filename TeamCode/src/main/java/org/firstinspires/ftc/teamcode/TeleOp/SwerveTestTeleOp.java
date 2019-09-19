package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "swerve test teleop")
public class SwerveTestTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = 1/1;
    public final static int SERVO_MAX_ANGLE = 190;
    Servo servoFrontLeft;
    @Override
    public void init() {
        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        double wheelAngle = joystickPositionToWheelAngle(joystickPositionX,joystickPositionY);
        double servoAngle = wheelServoAngle(wheelAngle);
        double servoPosition = servoAngleToPosition(servoAngle);
        servoFrontLeft.setPosition(servoPosition);

        telemetry.addData("wheelAngle",wheelAngle);
        telemetry.addData("servoAngle",servoAngle);
        telemetry.addData("servoPosition",servoPosition);
        telemetry.addData("actual servo position",servoFrontLeft.getPosition());
        telemetry.update();
    }

    public double wheelServoAngle(double wheelAngle){
        double servoAngle = WHEEL_SERVO_GEAR_RATIO * wheelAngle;
        return servoAngle;
    }

    public double servoAngleToPosition(double servoAngle){
       double servoPosition = servoAngle/SERVO_MAX_ANGLE;
       return servoPosition;
    }

    public double joystickPositionToWheelAngle (double joystickPositionX, double joystickPositionY){
        double wheelAngle = Math.atan2(joystickPositionY,joystickPositionX);
        wheelAngle = radiansDegreesTranslation(wheelAngle);
        wheelAngle = standardizedAngle(wheelAngle);
        return wheelAngle;
    }

    public double radiansDegreesTranslation (double radians){
        double degrees = radians * 180 / Math.PI;
        return degrees;

    }
    //Converting -180 to 180 to 0 to 360
    public double standardizedAngle(double angle) {
        return (angle + 360) % 360;
    }






}
