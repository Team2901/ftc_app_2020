package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.FileUtilities;

import java.util.ArrayList;
import java.util.List;

@SuppressLint("DefaultLocale")
@TeleOp(name = "Builder Skystone", group = "SKYSTONE")
public class BuilderSkystoneTeleOp extends OpMode {

    String CONFIG_FILENAME = "servo_offset_config.txt";

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;

    public ControlMode mode = ControlMode.ABSOLUTE;

    int servoUnderTestIndex;

    ArrayList<Servo> servoArrayList = new ArrayList<>();

    List<Double> offsets = new ArrayList<>();

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
        //robot.swerveStraight(0, 0);
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
                    mode = ControlMode.RELATIVE; break;
                case RELATIVE:
                    mode = ControlMode.OFFSET_SETTER; break;
                default:
                    mode = ControlMode.ABSOLUTE; break;
            }

            telemetry.speak(String.format("Mode %d %s", mode.ordinal(), mode.toString()));
        }

        telemetry.addData("Mode", mode);

        if (mode == ControlMode.ABSOLUTE || mode == ControlMode.RELATIVE) {

            // WHEEL CONTROL
            if (improvedGamepad1.right_stick_x.isPressed()) {

                double joyWheelAngle = improvedGamepad1.right_stick.getAngel();

                double rawPower;

                if (mode == ControlMode.ABSOLUTE) {
                    double angleCurrent = robot.getAngle();
                    double angleGoal = improvedGamepad1.right_stick.getAngel();

                    if (Math.abs(angleGoal - angleCurrent) > 5) {
                        rawPower = -robot.getCurrentTurnPower(robot.getAngle(), joyWheelAngle, 0, 1);
                    } else {
                        rawPower = 0;
                    }
                } else {
                    rawPower = improvedGamepad1.right_stick.getValue();
                }

                double power = getWheelPower(rawPower, gamepad1.left_bumper);
                robot.swerveTurn(power);

            } else if (improvedGamepad1.left_stick.isPressed()) {
                double power = getWheelPower(improvedGamepad1.left_stick.getValue(), gamepad1.left_bumper);
                double joyWheelAngle = improvedGamepad1.left_stick.getAngel();

                if (mode == ControlMode.ABSOLUTE) {
                    robot.swerveStraight(joyWheelAngle, power);
                } else {
                    robot.swerveStraightRelative(joyWheelAngle, power);
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

            telemetry.addData("", robot.frontLeftSwerveWheel.toString());
            telemetry.addData("", robot.frontRightSwerveWheel.toString());
            telemetry.addData("", robot.backLeftSwerveWheel.toString());
            telemetry.addData("", robot.backRightSwerveWheel.toString());

        } else {

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
                    offsets.set(servoUnderTestIndex, targetPosition);
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

        telemetry.update();
    }


    @Override
    public void stop() {

        super.stop();

        try {
            FileUtilities.writeConfigFile(CONFIG_FILENAME, offsets);
            telemetry.addData("Success writing to file", "");
        } catch (Exception e) {
            throw new RuntimeException("Error writing to file", e);
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
