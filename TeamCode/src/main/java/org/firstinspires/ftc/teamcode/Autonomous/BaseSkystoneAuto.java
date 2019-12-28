package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware.SERVO_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware.WHEEL_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware.WHEEL_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;


@Autonomous (name = "BaseAuto")
public class BaseSkystoneAuto extends LinearOpMode {

    public SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        swerveStraight(0, 0);




    }

    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = AngleUtilities.radiansDegreesTranslation(wheelAngleRad) - 90;
        return AngleUtilities.getPositiveNormalizedAngle(wheelAngle);
    }

    public double getPower(double x, double y, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return AngleUtilities.getRadius(x,y);
        }
    }

    public void swerveStraight(double joyWheelAngle, double power) {
        swerveMove(joyWheelAngle, joyWheelAngle, joyWheelAngle, joyWheelAngle, power);
    }

    public void swerveTurn(double power) {

        double fLAngle = joystickPositionToWheelAngle(-1, -1);
        double fRAngle = joystickPositionToWheelAngle(-1, 1);
        double bLAngle = joystickPositionToWheelAngle(1, -1);
        double bRAngle = joystickPositionToWheelAngle(1, 1);

        swerveMove(fLAngle, fRAngle, bLAngle, bRAngle, power);
    }

    public void swerveMove(double fLAngle, double fRAngle, double bLAngle, double bRAngle, double power) {

        angleCheck(fLAngle, robot.swerveWheels.frontLeftMotor);
        angleCheck(fRAngle, robot.swerveWheels.frontRightMotor);
        angleCheck(bLAngle, robot.swerveWheels.backLeftMotor);
        angleCheck(bRAngle, robot.swerveWheels.backRightMotor);

        double servoPositionfL = robot.swerveWheels.frontLeftMotor.wheelAngleToServoPosition();
        double servoPositionfR = robot.swerveWheels.frontRightMotor.wheelAngleToServoPosition();
        double servoPositionbL = robot.swerveWheels.backLeftMotor.wheelAngleToServoPosition();
        double servoPositionbR = robot.swerveWheels.backRightMotor.wheelAngleToServoPosition();

        robot.setWheelServoPosition(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

        double frontLeftPower = robot.swerveWheels.frontLeftMotor.modifier * power;
        double frontRightPower = robot.swerveWheels.frontRightMotor.modifier * power;
        double backLeftPower = robot.swerveWheels.backLeftMotor.modifier * power;
        double backRightPower = robot.swerveWheels.backRightMotor.modifier * power;

        robot.setWheelMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void angleCheck(double goal, SkystoneHardware.SwerveWheel swerveWheel) {

        double start = swerveWheel.targetAngle;

        goal = getNormalizedAngle(goal);

        double dAngleForward = getNormalizedAngle(goal - start);
        double targetAngleForward = dAngleForward + start;
        boolean forwardPossible = (WHEEL_MIN_ANGLE <= targetAngleForward && targetAngleForward <= WHEEL_MAX_ANGLE);

        double dAngleBackward = getNormalizedAngle(dAngleForward + 180);
        double targetAngleBackward = dAngleBackward + start;
        boolean backwardPossible = (WHEEL_MIN_ANGLE <= targetAngleBackward && targetAngleBackward <= WHEEL_MAX_ANGLE);

        boolean goForward;

        if (forwardPossible && backwardPossible) {
            goForward = (Math.abs(dAngleForward) < Math.abs(dAngleBackward));
        } else {
            goForward = forwardPossible;
        }

        double targetAngle;
        int modifier;

        if (goForward) {
            targetAngle = targetAngleForward;
            modifier = 1;

        } else {
            targetAngle = targetAngleBackward;
            modifier = -1;
        }

        swerveWheel.setTargetAndModifier(targetAngle, modifier);
    }

    public void turnWheels(double goalAngle){
        swerveMove(goalAngle,goalAngle,goalAngle,goalAngle, 0);
    }

    public void runToPosition( double goalPosition){

    }
}

