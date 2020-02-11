package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Builder Skystone", group = "SKYSTONE")
public class BuilderSkystoneTeleOp extends OpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;
    public static final int ABSOLUTE_MODE = 0;
    public static final int RELATIVE_MODE = 1;
    public int mode = ABSOLUTE_MODE;

    @Override
    public void init() {
        robot.init(hardwareMap);
        //robot.swerveStraight(0, 0);
        this.improvedGamepad1 = new ImprovedGamepad(gamepad1, timer, "g1");
        this.improvedGamepad2 = new ImprovedGamepad(gamepad2, timer, "g2");
    }

    @Override
    public void loop() {

        improvedGamepad1.update();
        improvedGamepad2.update();

        // WHEEL CONTROL
        if (improvedGamepad1.right_stick_x.isPressed()) {
            double power = getWheelPower(improvedGamepad1.right_stick.x.getValue(), gamepad1.left_bumper);
            robot.swerveTurn(power);
        } else if (improvedGamepad1.left_stick.isPressed()) {
            double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);
            double joyWheelAngle = improvedGamepad1.left_stick.getAngel();
            if (mode == ABSOLUTE_MODE) {
                robot.swerveStraightAbsolute(joyWheelAngle, power);
            }
            else {
                robot.swerveStraight(joyWheelAngle, power);
            }
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
            robot.crane.setPosition(robot.crane.getPosition() + .015);
        } else if (gamepad2.left_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() - .015);
        }
//WRIST CONTROL
        if (gamepad2.x) {
            robot.wrist.setPosition(robot.wrist.getPosition() + .01);
        } else if (gamepad2.y) {
            robot.wrist.setPosition(robot.wrist.getPosition() - .01);
        } else if (gamepad2.start) {
            robot.wrist.setPosition(.5);
        }
//JAW CONTROL
        if (gamepad2.a) {
            robot.jaw.setPosition(robot.jaw.getPosition() + .01);
        } else if (gamepad2.b) {
            robot.jaw.setPosition(robot.jaw.getPosition() - .01);
        }
//Waffle Grabber
        if (gamepad2.dpad_up) {
            robot.setGrabberPositition(.7, .84);
        } else if (gamepad2.dpad_down) {
            robot.setGrabberPositition(0, 0);
        }

        telemetry.addData("FL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.frontLeftMotor.targetAngle, robot.frontLeftMotor.modifier, robot.frontLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("FR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.frontRightMotor.targetAngle, robot.frontRightMotor.modifier, robot.frontRightMotor.wheelAngleToServoPosition()));
        telemetry.addData("BL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.backLeftMotor.targetAngle, robot.backLeftMotor.modifier, robot.backLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("BR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                robot.backRightMotor.targetAngle, robot.backRightMotor.modifier, robot.backRightMotor.wheelAngleToServoPosition()));

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
            return radius * robot.WHEEL_POWER_RATIO;
        }
    }
}
