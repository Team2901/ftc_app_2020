package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

import java.lang.reflect.Array;
import java.util.ArrayList;

@TeleOp (name = "Swerve Teleop")
public class SwerveTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = 1.0 / 4.0;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 12.75;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan((.5 * WIDTH_OF_ROBOT) / (.5 * LENGTH_OF_ROBOT));
    public final static int SERVO_MAX_ANGLE = 190;
    public final static double FRONT_LEFT_OFFSET = 0;
    public final static double BACK_LEFT_OFFSET = 0;
    public final static double FRONT_RIGHT_OFFSET = 0;
    public final static double BACK_RIGHT_OFFSET = 0;
    Servo servoFrontLeft;


    enum WheelPosition {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);

        int value;

        WheelPosition(int value) {
            this.value = value;
        }
    }

    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
            swerveTurn(gamepad1.right_stick_x);

        } else {
            setDriveServoPosition(joystickPositionX, joystickPositionY);
            setPower(joystickPositionX, joystickPositionY, 1);
        }

        telemetry.update();


    }

    /* This is a helper function that is used in 2 other methods.  This class takes the goal angle
    of "wheel angle" and shifts it to what the desired angle of the servo. This is the mathematical
    reason for the gear ratio
    */
    public double wheelAngleToServoAngle(double wheelAngle) {
        double servoAngle = WHEEL_SERVO_GEAR_RATIO * wheelAngle;
        telemetry.addData("Goal Angle" , wheelAngle);
        return servoAngle;
    }


    public double servoAngleToServoPosition(double servoAngle, WheelPosition wheelPosition) {
        switch (wheelPosition) {
            case BACK_LEFT:
                return servoAngleToServoPositionBL(servoAngle);

            case BACK_RIGHT:
                return servoAngleToServoPositionBR(servoAngle);

            case FRONT_LEFT:
                return servoAngleToServoPositionFL(servoAngle);

            case FRONT_RIGHT:
                default:
                return servoAngleToServoPositionFR(servoAngle);
        }
    }
    /*This is the next step in the process and takes the desired servo angle and divide it by the
    total servo angles so we can get a position between 0 and 1 for our desired location
     */
    public double servoAngleToServoPositionFL(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) +FRONT_LEFT_OFFSET;
        return servoPosition;
    }

    public double servoAngleToServoPositionFR(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) +FRONT_RIGHT_OFFSET;
        return servoPosition;
    }

    public double servoAngleToServoPositionBL(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) +BACK_LEFT_OFFSET;
        return servoPosition;
    }

    public double servoAngleToServoPositionBR(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE)+BACK_RIGHT_OFFSET;

        return servoPosition;
    }

    /*This method finds our desired angle based on the joysticks. We want out robot's wheels to
    follow the position of our joystick, so we find the angle of our joysticks position like it is
    a position on the coordinate plane
     */
    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = radiansDegreesTranslation(wheelAngleRad) - 90;
        double wheelAngleStandarized = standardizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }

    public double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;

    }

    //Converting -180 to 180 to 0 to 360
    public double standardizedAngle(double angle) {
        return (angle + 360) % 360;
    }

    public double getPower(double x, double y) {
        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return power;
    }

    public void setPower(double joystickPositionX, double joystickPositionY, double modifier) {

        double power = modifier * getPower(joystickPositionX, joystickPositionY);

        robot.backRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(power);

        telemetry.addData("Power", power);

    }

    public void setDriveServoPosition(double joystickPositionX, double joystickPositionY) {

        double servoPositionfL = joystickToServoPosition(joystickPositionX, joystickPositionY ,
                WheelPosition.FRONT_LEFT);
        double servoPositionfR = joystickToServoPosition(joystickPositionX, joystickPositionY ,
                WheelPosition.FRONT_RIGHT);
        double servoPositionbL = joystickToServoPosition(joystickPositionX, joystickPositionY ,
                WheelPosition.BACK_LEFT);
        double servoPositionbR = joystickToServoPosition(joystickPositionX, joystickPositionY ,
                WheelPosition.FRONT_RIGHT);


        setAllServos(servoPositionfL,servoPositionfR , servoPositionbL, servoPositionbR);

    }

    public void swerveTurn(double joyStickRightPosX) {

        double fRPos = 90 + TURN_ANGLE;
        double fLPos = 90 - TURN_ANGLE;
        double bLPos = 270 + TURN_ANGLE;
        double bRPos = 270 - TURN_ANGLE;

        setAllServos(wheelAngleToServoPosition(fRPos , WheelPosition.FRONT_RIGHT), wheelAngleToServoPosition(fLPos, WheelPosition.FRONT_LEFT),
                wheelAngleToServoPosition(bLPos , WheelPosition.BACK_LEFT), wheelAngleToServoPosition(bRPos , WheelPosition.BACK_RIGHT));

        setPower(joyStickRightPosX, 0, Math.signum(joyStickRightPosX));

    }

    public double joystickToServoPosition(double joystickPositionX, double joystickPositionY, WheelPosition wheelPosition) {
        double wheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
        double servoAngle = wheelAngleToServoAngle(wheelAngle);
        double servoPosition = servoAngleToServoPosition(servoAngle , wheelPosition);

        return servoPosition;

    }

    public double wheelAngleToServoPosition(double angle , WheelPosition wheelPosition) {
        double servoAngle = wheelAngleToServoAngle(angle);
        double servoPosition = servoAngleToServoPosition(servoAngle , wheelPosition);
        return servoPosition;
    }

    public void setAllServos(double fLPos, double fRPos, double bLPos, double bRPos) {
        robot.servoFrontRight.setPosition(fRPos);
        robot.servoBackRight.setPosition(bRPos);
        robot.servoFrontLeft.setPosition(fLPos);
        robot.servoBackLeft.setPosition(bLPos);

        telemetry.addData("Front Left Position" , fLPos);
        telemetry.addData("Front Right Position" , fRPos);
        telemetry.addData("Back Right Position" , bRPos);
        telemetry.addData(" Back Left Position" , bLPos);
    }
}