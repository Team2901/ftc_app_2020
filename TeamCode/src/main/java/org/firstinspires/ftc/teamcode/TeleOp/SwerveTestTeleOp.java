package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "swerve test teleop")
public class SwerveTestTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = 1.0/4;
    public final static int SERVO_MAX_ANGLE = 190;
    public final static double WHEEL_OFFSET = .17;
    Servo servoFrontLeft;
   // DcMotor frontLeft;


    @Override
    public void init() {
        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
 //       frontLeft = hardwareMap.dcMotor.get("frontLeft");
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        double wheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
        double servoAngle = wheelAngleToServoAngle(wheelAngle);
        double servoPosition = servoAngleToServoPosition(servoAngle);
        servoFrontLeft.setPosition(servoPosition);
       // frontLeft.setPower(getPower(joystickPositionX, joystickPositionY));

        telemetry.addData("wheelAngle", wheelAngle);
        telemetry.addData("servoAngle", servoAngle);
        telemetry.addData("servoPosition", servoPosition);
        telemetry.addData("actual servo position", servoFrontLeft.getPosition());
        telemetry.addData("power", getPower(joystickPositionX, joystickPositionY));
        telemetry.update();
    }

    public double wheelAngleToServoAngle(double wheelAngle) {
        double servoAngle = WHEEL_SERVO_GEAR_RATIO * wheelAngle;
        return servoAngle;
    }

    public double servoAngleToServoPosition(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE)+WHEEL_OFFSET;
        return servoPosition;
    }

    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngle = Math.atan2(joystickPositionY, joystickPositionX);
        wheelAngle = radiansDegreesTranslation(wheelAngle)-90;
        wheelAngle = standardizeAngle(wheelAngle);
        return wheelAngle;
    }

    public double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;
    }

    // Converting -180 to 180, 0 to 360
    public double standardizeAngle(double angle) {
        return (angle + 360) % 360;
    }

    public double getPower(double x, double y) {
        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return power;
    }


}
