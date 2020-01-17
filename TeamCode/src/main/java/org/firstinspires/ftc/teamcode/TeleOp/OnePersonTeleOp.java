package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

@SuppressLint("DefaultLocale")
@TeleOp(name= "One Person TeleOp", group = "SKYSTONE")
public class OnePersonTeleOp extends OpMode {


    public static final double WHEEL_POWER_RATIO = .5;
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
    public void loop() {


        improvedGamepad1.update();
        improvedGamepad2.update();

        boolean pause = improvedGamepad1.start.isPressed()|| improvedGamepad1.back.isPressed();




        // WHEEL CONTROL
        if (improvedGamepad1.right_stick_x.isPressed()) {
            double power = getWheelPower(improvedGamepad1.right_stick.x.getValue(), pause == true);
            robot.swerveTurn(power);
        } else if (improvedGamepad1.left_stick.isPressed()) {
            double power = getWheelPower(improvedGamepad1.left_stick.getValue(), /*gamepad1.left_bumper*/ pause == true);
            double joyWheelAngle = improvedGamepad1.left_stick.getAngel();
            robot.swerveStraight(joyWheelAngle, power);
        } else {
            robot.setWheelMotorPower(0, 0, 0, 0);
        }

        //LIFT CONTROL
        if (gamepad1.left_trigger > .2) {
            robot.lift.setPower(-.5);
        } else if (gamepad1.right_trigger > .2) {
            robot.lift.setPower(1);
        } else {
            robot.lift.setPower(0);
        }
//CRANE CONTROL
        if (gamepad1.right_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() + .005);
        } else if (gamepad1.left_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() - .005);
        }
//WRIST CONTROL
        if (gamepad1.x) {
            robot.wrist.setPosition(robot.wrist.getPosition() + .01);
        } else if (gamepad1.y) {
            robot.wrist.setPosition(robot.wrist.getPosition() - .01);
        }
//JAW CONTROL
        if (gamepad1.a) {
            robot.jaw.setPosition(robot.jaw.getPosition() + .01);
        } else if (gamepad1.b) {
            robot.jaw.setPosition(robot.jaw.getPosition() - .01);
        }
//Waffle Grabber
        if (gamepad1.dpad_up) {
            robot.setGrabberPositition(.7, .84);
        } else if (gamepad1.dpad_down) {
            robot.setGrabberPositition(0, 0);
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

    public double getWheelPower(double radius, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return radius * WHEEL_POWER_RATIO;
        }
    }
}
