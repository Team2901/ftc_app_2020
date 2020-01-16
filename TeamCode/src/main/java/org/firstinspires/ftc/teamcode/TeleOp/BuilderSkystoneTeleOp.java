package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Builder Skystone", group = "SKYSTONE")
public class BuilderSkystoneTeleOp extends OpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.right_stick_x) > .1) {
            double power = getPower(gamepad1.right_stick_x, 0, gamepad1.left_bumper);
            robot.swerveTurn(power);
        } else if (AngleUtilities.getRadius(joystickPositionX, joystickPositionY) > .2) {
            double power = getPower(joystickPositionX, joystickPositionY, gamepad1.left_bumper);
            double joyWheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            robot.swerveStraight(joyWheelAngle, power);
        } else {
            robot.setWheelMotorPower(0, 0, 0, 0);
        }

        //LIFT CONTROL
        if (gamepad2.left_trigger > .2) {
            robot.lift.setPower(-.5);
        } else if (gamepad2.right_trigger > .2) {
            robot.lift.setPower(1);
        } else {
            robot.lift.setPower(0);
        }
//CRANE CONTROL
        if (gamepad2.right_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() + .005);
        } else if (gamepad2.left_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() - .005);
        }
//WRIST CONTROL
        if (gamepad2.x) {
            robot.wrist.setPosition(robot.wrist.getPosition() + .01);
        } else if (gamepad2.y) {
            robot.wrist.setPosition(robot.wrist.getPosition() - .01);
        }
//JAW CONTROL
        if (gamepad2.a) {
            robot.jaw.setPosition(robot.jaw.getPosition() + .01);
        } else if (gamepad2.b) {
            robot.jaw.setPosition(robot.jaw.getPosition() - .01);
        }

        telemetry.addData("FL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.frontLeftMotor.targetAngle, robot.swerveWheels.frontLeftMotor.modifier, robot.swerveWheels.frontLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("FR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.frontRightMotor.targetAngle, robot.swerveWheels.frontRightMotor.modifier, robot.swerveWheels.frontRightMotor.wheelAngleToServoPosition()));
        telemetry.addData("BL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.backLeftMotor.targetAngle, robot.swerveWheels.backLeftMotor.modifier, robot.swerveWheels.backLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("BR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.backRightMotor.targetAngle, robot.swerveWheels.backRightMotor.modifier, robot.swerveWheels.backRightMotor.wheelAngleToServoPosition()));

        telemetry.addData("flo", robot.frontLeftOffset);
        telemetry.addData("fro", robot.frontRightOffset);
        telemetry.addData("blo", robot.backLeftOffset);
        telemetry.addData("bro", robot.backRightOffset);
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
            return AngleUtilities.getRadius(x, y);
        }
    }
}
