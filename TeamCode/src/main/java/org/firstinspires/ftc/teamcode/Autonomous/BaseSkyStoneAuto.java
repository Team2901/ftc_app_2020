package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.MotoLinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware.LABEL_SKY_BUTTER;

@SuppressLint("DefaultLocale")
public abstract class BaseSkyStoneAuto extends MotoLinearOpMode {

    public static final int GO_TO_ANGLE_BUFFER = 7;
    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();

    public void turnTo(double angle) {
        turnTo(angle, .5);
    }

    public void turnTo(double angle, double power) {
        goToAngle(robot.getAngle(), angle, power);
    }

    public void moveInches(double angle, double inches, double power) {

        robot.swerveStraight(angle, 0);
        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setWheelTargetPositions((int) (inches * robot.inchesToEncoder));

        robot.swerveStraight(angle, power);
        while (robot.wheelsAreBusy() && opModeIsActive()) {
            telemetry.addData("FL", String.format("angle: %.2f, mod: %d, pos: %d",
                    robot.frontLeftSwerveWheel.targetAngle, robot.frontLeftSwerveWheel.modifier, robot.frontLeft.getCurrentPosition()));
            telemetry.addData("FR", String.format("angle: %.2f, mod: %d, pos: %d",
                    robot.frontRightSwerveWheel.targetAngle, robot.frontRightSwerveWheel.modifier, robot.frontRight.getCurrentPosition()));
            telemetry.addData("BL", String.format("angle: %.2f, mod: %d, pos: %d",
                    robot.backLeftSwerveWheel.targetAngle, robot.backLeftSwerveWheel.modifier, robot.backLeft.getCurrentPosition()));
            telemetry.addData("BR", String.format("angle: %.2f, mod: %d, pos: %d",
                    robot.backRightSwerveWheel.targetAngle, robot.backRightSwerveWheel.modifier, robot.backRight.getCurrentPosition()));

            telemetry.update();
        }

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.swerveStraight(angle, 0);
    }

    public void goToAngle(double angleStart, double angleGoal, double power) {

        double angleCurrent = angleStart;

        while ((Math.abs(angleGoal - angleCurrent) > GO_TO_ANGLE_BUFFER) && opModeIsActive()) {
            angleCurrent = robot.getAngle();
            double powerCurrent = getCurrentTurnPower(angleCurrent, angleGoal, angleStart, power);
            robot.swerveTurn(-powerCurrent);

            telemetry.addData("Start Angle ", "%.1f", angleStart);
            telemetry.addData("Goal Angle  ", "%.1f", angleGoal);
            telemetry.addData("Cur Angle   ", "%.1f", angleCurrent);
            telemetry.addData("Remain Angle", "%.1f", AngleUnit.normalizeDegrees(angleGoal - angleCurrent));
            telemetry.addData("Power       ", "%.2f", powerCurrent);
            telemetry.update();

        }
        robot.swerveTurn(0);
        telemetry.addData("Is Stopped", "");
        telemetry.update();
    }

    public double getCurrentTurnPower(double absCurrent, double absGoal, double absStart, double maxPower) {
        double relCurrent = AngleUtilities.getNormalizedAngle(absCurrent - absStart);
        double relGoal = AngleUtilities.getNormalizedAngle(absGoal - absStart);
        double remainingDistance = AngleUtilities.getNormalizedAngle(relGoal - relCurrent);

        double basePower = 0.025 * remainingDistance;
        double stallPower = 0.05 * Math.signum(remainingDistance);

        return Range.clip(basePower + stallPower, -Math.abs(maxPower), Math.abs(maxPower));
    }

    public void platformParkInner(int team) {
        double colorAngle;
        if (team == Color.RED) {
            colorAngle = robot.ROBOT_LEFT_ANGLE;
        } else {
            colorAngle = robot.ROBOT_RIGHT_ANGLE;

        }
        //Step one: turn wheels 90 degrees counterclockwise and go forward 28.5 inches and lower grabbers.
        moveInches(colorAngle, 28.5, 0.5);
        robot.rightGrabber.setPosition(robot.GRABBER_MAX);
        robot.leftGrabber.setPosition(robot.GRABBER_MAX);
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 500) ;

        //Step two: back up 26 inches and raise grabbers.
        moveInches(colorAngle, -26, 0.5);
        robot.rightGrabber.setPosition(robot.GRABBER_MIN);
        robot.leftGrabber.setPosition(robot.GRABBER_MIN);
        //Step three: turn wheels 90 degrees counterclockwise and slide out 2 ft.
        moveInches(robot.ROBOT_FRONT_ANGLE, -24, 0.5);
        //Step four: turn wheels to position 90, move forwards 2 ft
        moveInches(colorAngle, 24, 0.5);
        //Step Five: Turn Wheels to 0, move forward 2 ft
        moveInches(robot.ROBOT_FRONT_ANGLE, -24, 0.5);
        //Step  Six: Stop
    }

    public Float findSkyStone() {

        Float centerPercentDifference = null;
        float stonePercentLocation = 0;
        if (robot.webCamera.tfod == null) {
            return (0.0f);
        }

        List<Recognition> updatedRecognitions = robot.webCamera.tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                //If x > 380, the skystone is in position three. (Three away from the edge) If x > 620 it is at position 2, and if x > 350 it is in position 1
                if (!recognition.getLabel().equals(LABEL_SKY_BUTTER)) {
                    continue;
                }

                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                int centerFrame = recognition.getImageWidth() / 2;
                float centerSkyStone = (recognition.getRight() + recognition.getLeft()) / 2;
                telemetry.addData("Center Frame", centerFrame);
                telemetry.addData("Center Stone", centerSkyStone);
                float centerDifference = centerSkyStone - centerFrame;
                telemetry.addData("Difference", centerDifference);
                centerPercentDifference = (centerDifference / centerFrame) * 100;
                telemetry.addData("Percent Difference", centerPercentDifference);
                stonePercentLocation = (centerSkyStone / recognition.getImageWidth() * 100);

            }
        }
        telemetry.addData("Skystones Location Debugger", stonePercentLocation);
        telemetry.update();
        return centerPercentDifference;
    }

    public void SkytoneScanner(int red) {
        Float skyStonePosition = findSkyStone();

        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());
        while ((skyStonePosition == null || Math.abs(skyStonePosition) < 1) && opModeIsActive()) {
            telemetry.addData("loop is running", "");
            telemetry.update();

            robot.swerveStraight(0, .2);

            skyStonePosition = findSkyStone();
        }
        telemetry.addData("out of loop", "");
        telemetry.update();

        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = Math.abs(fLeftEnd - fLeftStart);
        robot.swerveStraight(0, 0);
        telemetry.addData("Start ", fLeftStart);
        telemetry.addData("End ", fLeftEnd);
        telemetry.addData("Diff ", diff);
        telemetry.addData("location", skyStonePosition);
        telemetry.update();

        //turning and grabbing the skystone

        this.turnTo(90);

        robot.jaw.setPosition(robot.OPEN_JAW);

        moveInches(0, 6, .5);

        robot.jaw.setPosition(robot.CLOSED_JAW);

        //lifting skystone off the ground by .5 in

        int targetPosition = 72;

        robot.lift.setTargetPosition(targetPosition);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.5);

        this.moveInches(0, -4, .5);

        //turning robot 90 degrees clockwise

        this.turnTo(90);
    }

    public void initAndActivateWebCameraWithTensorFlow() {

        // Init the web camera with TensorFlow
        robot.initWebCamera(hardwareMap);

        // Activate TensorFlow
        robot.webCamera.activateTfod();

        // Check for errors
        if (robot.webCamera.hasError()) {
            telemetry.addData("Failed!", robot.webCamera.errorMessage);
        } else {
            telemetry.addData("Successful!", "");
        }
    }
}

