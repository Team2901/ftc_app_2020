package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Builder Skystone", group = "SKYSTONE")
public class BuilderSkystoneTeleOp extends OpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;
    public static final int ABSOLUTE_MODE = 0;
    public static final int RELATIVE_MODE = 1;
    public static final int OFFSET_MODE = 2;
    public int mode = RELATIVE_MODE;
    Servo servoUnderTest;
    int servoIndex;
    public String[] driveModeNames = {"ABSOLUTE_MODE", "RELATIVE_MODE", "OFFSET_MODE"};

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

        if(this.improvedGamepad1.dpad_up.isInitialPress() || this.improvedGamepad1.dpad_down.isInitialPress()){

            // Update the offsets when leaving the mode
            if (mode == OFFSET_MODE) {
                robot.swerveWheels[0].setOffset(robot.swerveWheels[0].servo.getPosition());
                robot.swerveWheels[1].setOffset(robot.swerveWheels[1].servo.getPosition());
                robot.swerveWheels[2].setOffset(robot.swerveWheels[2].servo.getPosition());
                robot.swerveWheels[3].setOffset(robot.swerveWheels[3].servo.getPosition());
            }

            if (this.improvedGamepad1.dpad_up.isInitialPress()) {
                mode++;
                if(mode > 2){
                    mode = 0;
                }
            } else {
                mode--;
                if (mode < 0) {
                    mode = 2;
                }
            }

            if (mode == OFFSET_MODE) {
                robot.swerveStraight(0,0);
            }
        }

        telemetry.addData("Current Drive Mode", driveModeNames[mode]);

        if (mode == ABSOLUTE_MODE) {

            if (improvedGamepad1.right_stick.isPressed()
                    || improvedGamepad1.a.isPressed()
                    || improvedGamepad1.b.isPressed()
                    || improvedGamepad1.x.isPressed()
                    || improvedGamepad1.y.isPressed()) {

                final double maxPower;
                final double angleGoal;
                if (improvedGamepad1.a.isPressed()) {
                    angleGoal = 180;
                    maxPower = .5;
                } else if (improvedGamepad1.b.isPressed()) {
                    angleGoal = -90;
                    maxPower = .5;
                } else if (improvedGamepad1.x.isPressed()) {
                    angleGoal = 90;
                    maxPower = .5;
                } else if (improvedGamepad1.y.isPressed()) {
                    angleGoal = 0;
                    maxPower = .5;
                } else {
                    angleGoal = improvedGamepad1.right_stick.getAngel();
                    maxPower = improvedGamepad1.right_stick.getValue();
                }

                double angleCurrent = robot.getAngle();

                double rawPower;
                if (Math.abs(angleGoal - angleCurrent) > 2) {
                    rawPower = robot.getCurrentTurnPower(robot.getAngle(), angleGoal, maxPower);
                } else {
                    rawPower = 0;
                }

                double power = getWheelPower(rawPower, gamepad1.left_bumper);

                telemetry.addData("angleGoal", angleGoal);
                telemetry.addData("angleCurrent", angleCurrent);
                telemetry.addData("rawPower", rawPower);
                telemetry.addData("turn power", power);

                robot.swerveTurn(power);
            } else if (improvedGamepad1.left_stick.isPressed()) {
                double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);
                double joyWheelAngle = improvedGamepad1.left_stick.getAngel();
                robot.swerveStraightAbsolute(joyWheelAngle, power);
            } else {
                robot.setWheelMotorPower(0, 0, 0, 0);
            }
        } else if (mode == RELATIVE_MODE) {

            // WHEEL CONTROL
            if (improvedGamepad1.right_stick_x.isPressed()) {
                double power = getWheelPower(improvedGamepad1.right_stick.x.getValue(), gamepad1.left_bumper);
                robot.swerveTurn(power);

            } else if (improvedGamepad1.left_stick.isPressed()) {
                double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);
                double joyWheelAngle = improvedGamepad1.left_stick.getAngel();
                robot.swerveStraight(joyWheelAngle, power);

            } else {
                robot.setWheelMotorPower(0, 0, 0, 0);
            }
        } else if (mode == OFFSET_MODE) {

            robot.setWheelMotorPower(0, 0, 0, 0);

            improvedGamepad1.update();

            if(this.improvedGamepad1.dpad_right.isInitialPress()){
                servoIndex++;
                if(servoIndex >= robot.swerveWheels.length){
                    servoIndex = 0;
                }
            } else if(this.improvedGamepad1.dpad_left.isInitialPress()) {
                servoIndex--;
                if(servoIndex < 0){
                    servoIndex = robot.swerveWheels.length - 1;
                }
            }

            servoUnderTest = robot.swerveWheels[servoIndex].servo;

            if(servoUnderTest != null){
                if(this.improvedGamepad1.left_bumper.isInitialPress()){
                    servoUnderTest.setPosition(servoUnderTest.getPosition()-0.1);
                } else if(this.improvedGamepad1.right_bumper.isInitialPress()){
                    servoUnderTest.setPosition(servoUnderTest.getPosition()+0.1);
                } else if(this.improvedGamepad1.left_trigger.isInitialPress()){
                    servoUnderTest.setPosition(servoUnderTest.getPosition()-0.01);
                } else if(this.improvedGamepad1.right_trigger.isInitialPress()) {
                    servoUnderTest.setPosition(servoUnderTest.getPosition()+0.01);
                }

                telemetry.addData("Left bumper","-0.1");
                telemetry.addData("Right bumper","+0.1");
                telemetry.addData("Left trigger","-0.01");
                telemetry.addData("Right trigger","+0.01");
                telemetry.addData("Position", servoUnderTest.getPosition());
                telemetry.addData("Servo name",robot.swerveWheels[servoIndex].name);
            }

            telemetry.addData("D Pad Right/Left", "Increment/decrement servos");
        } else {
            robot.setWheelMotorPower(0, 0, 0, 0);
        }

        //LIFT CONTROL
        if (gamepad2.left_trigger > .2) {
            robot.lift.setPower(-1);
        } else if (gamepad2.right_trigger > .2) {
            // Don't let the lift go below 0 encoder ticks
            if (robot.lift.getCurrentPosition() > 0) {
                robot.lift.setPower(.5);
            } else {
                robot.lift.setPower(0);
            }
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

        telemetry.addData("", robot.frontLeftSwerveWheel.toString());
        telemetry.addData("", robot.frontRightSwerveWheel.toString());
        telemetry.addData("", robot.backLeftSwerveWheel.toString());
        telemetry.addData("", robot.backRightSwerveWheel.toString());
        telemetry.addData("lift pos", robot.lift.getCurrentPosition());

        telemetry.update();
    }

    public double getWheelPower(double radius, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return radius * robot.WHEEL_POWER_RATIO;
        }
    }

    @Override
    public void stop() {

        super.stop();

        String errorMsg = robot.writeOffsets();
        if (errorMsg != null) {
            throw new RuntimeException(errorMsg);
        }
    }
}
