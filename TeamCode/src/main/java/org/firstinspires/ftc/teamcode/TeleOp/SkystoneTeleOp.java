package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Skystone", group = "SKYSTONE")
public class SkystoneTeleOp extends OpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        this.improvedGamepad1 = new ImprovedGamepad(gamepad1, timer, "g1");
        this.improvedGamepad2 = new ImprovedGamepad(gamepad2, timer, "g2");
    }

    @Override
    public void loop()  {
        improvedGamepad1.update();
        improvedGamepad2.update();

        // WHEEL CONTROL
        if (improvedGamepad1.right_stick_x.isPressed()) {
            double power = getWheelPower(improvedGamepad1.right_stick.x.getValue(), gamepad1.left_bumper);
            robot.swerveTurn(power);
        } else if (improvedGamepad1.left_stick.isPressed()) {
            double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);
            double joyWheelAngle = improvedGamepad1.left_stick.getAngel();
            robot.swerveStraight(joyWheelAngle, power);
        } else {
            robot.setWheelMotorPower(0,0,0,0);
        }

        // GRABBER CONTROL
        if (improvedGamepad1.dpad_up.isPressed()) {
            robot.leftGrabber.setPosition(robot.leftGrabber.getPosition()+.01);
            robot.rightGrabber.setPosition(robot.rightGrabber.getPosition()+.01);
        } else if (improvedGamepad1.dpad_down.isPressed()) {
            robot.leftGrabber.setPosition(robot.leftGrabber.getPosition()-.01);
            robot.rightGrabber.setPosition(robot.rightGrabber.getPosition()-.01);
        }

        //CRANE CONTROL
        if (improvedGamepad2.right_stick.y.getValue() > 0) {
            robot.crane.setPosition(robot.crane.getPosition()+.01);
        } else if (improvedGamepad2.right_stick.y.getValue() < 0) {
            robot.crane.setPosition(robot.crane.getPosition()-.01);
        }

        //LIFT CONTROL
        if (improvedGamepad2.left_stick.y.isPressed()) {
            robot.lift.setPower(improvedGamepad2.left_stick.y.getValue());
        } else {
            robot.lift.setPower(0);
        }

        // TODO use a/b/x/y to move lift up/down left amounts
        
        //JAW CONTROL
        if (gamepad2.right_bumper) {
            robot.jaw.setPosition(robot.crane.getPosition() + .005);
        } else if (gamepad2.left_bumper) {
            robot.jaw.setPosition(robot.crane.getPosition() - .005);
        }
        //WRIST CONTROL
        if (improvedGamepad2.right_trigger.isPressed()) {
            robot.wrist.setPosition(robot.wrist.getPosition() + .01);
        } else if (improvedGamepad2.left_trigger.isPressed()) {
            robot.wrist.setPosition(robot.wrist.getPosition() - .01);
        }


        telemetry.addData("FL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.frontLeftMotor.targetAngle, robot.swerveWheels.frontLeftMotor.modifier, robot.swerveWheels.frontLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("FR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.frontRightMotor.targetAngle, robot.swerveWheels.frontRightMotor.modifier, robot.swerveWheels.frontRightMotor.wheelAngleToServoPosition()));
        telemetry.addData("BL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.backLeftMotor.targetAngle, robot.swerveWheels.backLeftMotor.modifier, robot.swerveWheels.backLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("BR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.swerveWheels.backRightMotor.targetAngle, robot.swerveWheels.backRightMotor.modifier, robot.swerveWheels.backRightMotor.wheelAngleToServoPosition()));

        telemetry.addData("frontLeftMotor", String.format("min: %f max:%f", robot.swerveWheels.frontLeftMotor.minWheelAngle, robot.swerveWheels.frontLeftMotor.maxWheelAngle));
        telemetry.addData("frontRightMotor", String.format("min: %f max:%f", robot.swerveWheels.frontRightMotor.minWheelAngle, robot.swerveWheels.frontRightMotor.maxWheelAngle));
        telemetry.addData("backLeftMotor", String.format("min: %f max:%f", robot.swerveWheels.backLeftMotor.minWheelAngle, robot.swerveWheels.backLeftMotor.maxWheelAngle));
        telemetry.addData("backRightMotor", String.format("min: %f max:%f", robot.swerveWheels.backRightMotor.minWheelAngle, robot.swerveWheels.backRightMotor.maxWheelAngle));

        telemetry.addData("x  ", improvedGamepad1.left_stick.x.getValue());
        telemetry.addData("y  ", improvedGamepad1.left_stick.y.getValue());
        telemetry.addData("rad", improvedGamepad1.left_stick.isPressed());
        telemetry.addData("rad", improvedGamepad1.left_stick.getValue());
        telemetry.addData("ang", improvedGamepad1.left_stick.getAngel());


        telemetry.addData("right_stick_x", improvedGamepad1.right_stick.x.getValue());
        telemetry.update();
    }

    public double getWheelPower(double radius, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return radius * robot.wheelPowerRatio;
        }
    }
}

