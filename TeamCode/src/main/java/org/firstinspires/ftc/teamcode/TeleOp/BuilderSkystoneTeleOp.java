package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;

@SuppressLint("DefaultLocale")
@TeleOp(name = "BUILDER SKYSTONE TELEOP", group = "competition")
public class BuilderSkystoneTeleOp extends OpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        swerveStraight(0, 0);
    }

    @Override
    public void loop()  {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.right_stick_x) > .1) {
            double power = getPower(gamepad1.right_stick_x, 0, gamepad1.left_bumper);
            swerveTurn(power);
        } else if (AngleUtilities.getRadius(joystickPositionX, joystickPositionY) > .2) {
            double power = getPower(joystickPositionX, joystickPositionY, gamepad1.left_bumper);
            double joyWheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            swerveStraight(joyWheelAngle, power);
        } else {
            robot.setWheelMotorPower(0,0,0,0);
        }

        telemetry.addData("FL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.frontLeftMotor.targetAngle, robot.swerveWheels.frontLeftMotor.modifier, robot.swerveWheels.frontLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("FR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.frontRightMotor.targetAngle, robot.swerveWheels.frontRightMotor.modifier, robot.swerveWheels.frontRightMotor.wheelAngleToServoPosition()));
        telemetry.addData("BL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.backLeftMotor.targetAngle, robot.swerveWheels.backLeftMotor.modifier, robot.swerveWheels.backLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("BR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.backRightMotor.targetAngle, robot.swerveWheels.backRightMotor.modifier, robot.swerveWheels.backRightMotor.wheelAngleToServoPosition()));

        telemetry.update();
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

    public void angleCheck(double goal, BuilderSkystoneHardware.SwerveWheel swerveWheel) {

        double start = swerveWheel.targetAngle;

        goal = getNormalizedAngle(goal);

        double dAngleForward = getNormalizedAngle(goal - start);
        double targetAngleForward = dAngleForward + start;
        boolean forwardPossible = (BuilderSkystoneHardware.WHEEL_MIN_ANGLE <= targetAngleForward && targetAngleForward <= BuilderSkystoneHardware.WHEEL_MAX_ANGLE);

        double dAngleBackward = getNormalizedAngle(dAngleForward + 180);
        double targetAngleBackward = dAngleBackward + start;
        boolean backwardPossible = (BuilderSkystoneHardware.WHEEL_MIN_ANGLE <= targetAngleBackward && targetAngleBackward <= BuilderSkystoneHardware.WHEEL_MAX_ANGLE);

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
}
