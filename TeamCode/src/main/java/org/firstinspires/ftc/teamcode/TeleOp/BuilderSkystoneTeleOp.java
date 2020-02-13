package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.FileUtilities;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Builder Skystone Mtoebes", group = "SKYSTONE")
public class BuilderSkystoneTeleOp extends OpMode {

    String CONFIG_FILENAME = "servo_offset_config.txt";

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;

    public ControlMode mode = ControlMode.ABSOLUTE;
    int servoUnderTestIndex;

    public enum ControlMode {
        ABSOLUTE, RELATIVE, OFFSET_SETTER;

        @Override
        public String toString() {

            switch (this) {
                case ABSOLUTE:
                    return "Absolute";
                case RELATIVE:
                    return "Relative";
                default:
                    return "Offset Setter";
            }
        }
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        this.improvedGamepad1 = new ImprovedGamepad(gamepad1, timer, "g1");
        this.improvedGamepad2 = new ImprovedGamepad(gamepad2, timer, "g2");
    }

    @Override
    public void loop() {

        improvedGamepad1.update();
        improvedGamepad2.update();

        if (improvedGamepad1.start.isInitialPress()) {

            switch (mode) {
                case ABSOLUTE:
                    mode = ControlMode.RELATIVE;
                    break;
                case RELATIVE:
                    mode = ControlMode.OFFSET_SETTER;
                    break;
                default:
                    mode = ControlMode.ABSOLUTE;
                    break;
            }

            telemetry.speak(String.format("Mode %d %s", mode.ordinal(), mode.toString()));
        }

        telemetry.addData("Mode", mode);

        if (mode == ControlMode.ABSOLUTE) {

            if (improvedGamepad1.right_stick.isPressed()
                    || improvedGamepad1.a.isPressed()
                    || improvedGamepad1.b.isPressed()
                    || improvedGamepad1.x.isPressed()
                    || improvedGamepad1.y.isPressed()) {

                final double angleGoal;
                if (improvedGamepad1.a.isPressed()) {
                    angleGoal = 180;
                } else if (improvedGamepad1.b.isPressed()) {
                    angleGoal = -90;
                } else if (improvedGamepad1.x.isPressed()) {
                    angleGoal = 90;
                } else if (improvedGamepad1.y.isPressed()) {
                    angleGoal = 0;
                } else {
                    angleGoal = improvedGamepad1.right_stick.getAngel();
                }

                double angleCurrent = robot.getAngle();

                double rawPower;
                if (Math.abs(angleGoal - angleCurrent) > 5) {
                    rawPower = -robot.getCurrentTurnPower(robot.getAngle(), angleGoal, 0, 1);
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

                double leftSickAngle = improvedGamepad1.left_stick.getAngel();
                double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);

                telemetry.addData("leftStickAngle", leftSickAngle);
                telemetry.addData("straight power", power);

                robot.swerveStraight(leftSickAngle, power);

            } else {
                robot.setWheelMotorPower(0, 0, 0, 0);
            }
        } else if (mode == ControlMode.RELATIVE) {

            // WHEEL CONTROL
            if (improvedGamepad1.right_stick.isPressed()) {

                double rawPower = improvedGamepad1.right_stick.getValue();
                double power = getWheelPower(rawPower, gamepad1.left_bumper);

                telemetry.addData("rawPower", rawPower);
                telemetry.addData("turn power", power);

                robot.swerveTurn(power);

            } else if (improvedGamepad1.left_stick.isPressed()) {

                double leftSickAngle = improvedGamepad1.left_stick.getAngel();
                double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);

                telemetry.addData("leftStickAngle", leftSickAngle);
                telemetry.addData("straight power", power);

                robot.swerveStraightRelative(leftSickAngle, power);

            } else {
                robot.setWheelMotorPower(0, 0, 0, 0);
            }

        } else if (mode == ControlMode.OFFSET_SETTER) {

            robot.setWheelMotorPower(0, 0, 0, 0);

            boolean underTestChange = false;

            if (this.improvedGamepad1.dpad_right.isInitialPress()) {
                servoUnderTestIndex++;
                underTestChange = true;
                if (servoUnderTestIndex >= robot.allSwerveWheels.length) {
                    servoUnderTestIndex = 0;
                }
            } else if (this.improvedGamepad1.dpad_left.isInitialPress()) {
                servoUnderTestIndex--;
                underTestChange = true;
                if (servoUnderTestIndex < 0) {
                    servoUnderTestIndex = robot.allSwerveWheels.length - 1;
                }
            }

            final BaseSkyStoneHardware.SwerveWheel swerveWheelUnderTest = robot.allSwerveWheels[servoUnderTestIndex];

            if (underTestChange) {
                telemetry.speak(swerveWheelUnderTest.servoConfigName);
            } else {
                double targetPosition = swerveWheelUnderTest.servo.getPosition();

                if (this.improvedGamepad1.a.isInitialPress()) {
                    swerveWheelUnderTest.setOffset(targetPosition);
                    robot.offsets.set(servoUnderTestIndex, targetPosition);
                    telemetry.speak("Offset Updated");
                } else if (this.improvedGamepad1.left_bumper.isInitialPress()) {
                    targetPosition -= 0.1;
                } else if (this.improvedGamepad1.right_bumper.isInitialPress()) {
                    targetPosition += 0.1;
                } else if (this.improvedGamepad1.left_trigger.isInitialPress()) {
                    targetPosition -= 0.01;
                } else if (this.improvedGamepad1.right_trigger.isInitialPress()) {
                    targetPosition += 0.01;
                }

                swerveWheelUnderTest.servo.setPosition(targetPosition);
            }

            telemetry.addData("Servo name", swerveWheelUnderTest.servoConfigName);
            telemetry.addData("Position", swerveWheelUnderTest.servo.getPosition());
            telemetry.addData("Left/Right bumper", "-/+ 0.1");
            telemetry.addData("Left/Right trigger", "-/+ 0.01");
            telemetry.addData("A", "Update offset");
            telemetry.addData("D Pad Right/Left", "Increment/decrement servos");
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

        telemetry.addData("Angle", robot.getAngle());
        telemetry.addData("", robot.frontLeftSwerveWheel.toString());
        telemetry.addData("", robot.frontRightSwerveWheel.toString());
        telemetry.addData("", robot.backLeftSwerveWheel.toString());
        telemetry.addData("", robot.backRightSwerveWheel.toString());

        telemetry.addData("lift", String.format("power: %.2f encoder: %d", robot.lift.getPower(), robot.lift.getCurrentPosition()));
        telemetry.addData("crane", String.format("pos: %.2f", robot.crane.getPosition()));
        telemetry.addData("wrist", String.format("pos: %.2f", robot.wrist.getPosition()));
        telemetry.addData("jaw", String.format("pos: %.2f", robot.jaw.getPosition()));
        telemetry.addData("grabber", String.format("L_pos: %.2f R_pos: %.2f", robot.leftGrabber.getPosition(), robot.rightGrabber.getPosition()));

        telemetry.update();
    }

    @Override
    public void stop() {

        super.stop();

        String errorMessage = robot.writeOffsets();

       if (errorMessage != null) {
            throw new RuntimeException(errorMessage);
        }
    }

    public double getWheelPower(double radius, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return radius * robot.wheelPowerRatio;
        }
    }
}
