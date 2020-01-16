package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import java.util.List;

@SuppressLint("DefaultLocale")
public abstract class BaseSkyStoneAuto extends LinearOpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public static final int GO_TO_ANGLE_BUFFER = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);

        waitForStart();

        while (opModeIsActive()) {

        }
    }

    public void turnTo (double angle){

        goToAngle(robot.getAngle(), angle);
    }

    public void moveInches (double angle, double inches, double power){

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setWheelTargetPositions((int)(-inches*robot.inchesToEncoder));

        robot.swerveStraight(angle,power);

        while (robot.wheelsAreBusy() && opModeIsActive());

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.swerveStraight( angle,0);
    }

    public void goToAngle(double angleStart, double angleGoal) {

        double angleCurrent = angleStart;

        while ((Math.abs(angleGoal - angleCurrent) > GO_TO_ANGLE_BUFFER) && opModeIsActive()) {
            angleCurrent = robot.getAngle();
            double power = getPower(angleCurrent, angleGoal, angleStart);
            robot.swerveTurn(power);

            telemetry.addData("Start Angle ", "%.1f", angleStart);
            telemetry.addData("Goal Angle  ", "%.1f", angleGoal);
            telemetry.addData("Cur Angle   ", "%.1f", angleCurrent);
            telemetry.addData("Remain Angle", "%.1f", AngleUnit.normalizeDegrees(angleGoal - angleCurrent));
            telemetry.addData("Power       ", "%.2f", power);
            telemetry.update();

        }
        robot.swerveTurn(0);
        telemetry.addData("Is Stopped" , "");
        telemetry.update();
    }

    public double getPower(double absCurrent, double absGoal, double absStart) {
        double relCurrent = AngleUtilities.getNormalizedAngle(absCurrent - absStart);
        double relGoal = AngleUtilities.getNormalizedAngle(absGoal - absStart);
        double remainingDistance = AngleUtilities.getNormalizedAngle(relGoal - relCurrent);

        double basePower = 0.045 * remainingDistance;
        double stallPower = 0.1 * Math.signum(remainingDistance);
        return Range.clip(basePower + stallPower, -1, 1);
    }

    public void platformParkInner(int team){
        double colorAngle;
        if (team == Color.RED){
            colorAngle = robot.ROBOT_LEFT_ANGLE;
        }
        else {
            colorAngle = robot.ROBOT_RIGHT_ANGLE;

        }
        //Step one: turn wheels 90 degrees counterclockwise and go forward 28.5 inches and lower grabbers.
        moveInches(colorAngle,28.5 , 0.5 );
        robot.rightGrabber.setPosition (robot.GRABBER_MAX);
        robot.leftGrabber.setPosition(robot.GRABBER_MAX);
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds()< 500);

        //Step two: back up 26 inches and raise grabbers.
        moveInches(colorAngle,-26 , 0.5);
        robot.rightGrabber.setPosition (robot.GRABBER_MIN);
        robot.leftGrabber.setPosition(robot.GRABBER_MIN);
        //Step three: turn wheels 90 degrees counterclockwise and slide out 2 ft.
        moveInches(robot.ROBOT_FRONT_ANGLE, -24  , 0.5);
        //Step four: turn wheels to position 90, move forwards 2 ft
        moveInches(colorAngle, 24, 0.5);
        //Step Five: Turn Wheels to 0, move forward 2 ft
        moveInches(robot.ROBOT_FRONT_ANGLE, -24, 0.5);
        //Step  Six: Stop
    }

    public float findSkyStone() {

        float centerPercentDifference = 0;
        float stonePercentLocation = 0;

        if (robot.webCamera.tfod == null) {
            return 50;
        }

        List<Recognition> updatedRecognitions = robot.webCamera.tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                //If x > 380, the skystone is in position three. (Three away from the edge) If x > 620 it is at position 2, and if x > 350 it is in position 1
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                int centerFrame = recognition.getImageWidth()/2;
                float centerStone = (recognition.getRight() + recognition.getLeft()) / 2;
                telemetry.addData("Center Frame", centerFrame);
                telemetry.addData("Center Stone", centerStone);
                float centerDifference = centerStone - centerFrame;
                telemetry.addData("Difference", centerDifference);
                centerPercentDifference = (centerDifference / centerFrame) * 100;
                telemetry.addData("Percent Difference", centerPercentDifference);
                stonePercentLocation = (centerStone / recognition.getImageWidth() * 100);
                if (recognition.getLabel().equals(robot.LABEL_SKY_BUTTER))
                    break;
            }
        }
        telemetry.addData("Skystones Location Debugger", stonePercentLocation);
        telemetry.update();
        return stonePercentLocation;
    }


    public void SkystonsScanner(int red) {
        float skyStonePosition = findSkyStone();

        // skyStoneGridLocation = convertPositionToGridOffset(skyStonePosition);


        //Step 2 Get Skystone

        while(skyStonePosition < 1){
            telemetry.addData("loop is running", "");
            telemetry.update();

            robot.setWheelMotorPower(-.5,-.5,-.5,-.5);

            skyStonePosition = findSkyStone();
        }

        this.goToAngle(0, 90);

        robot.jaw.setPosition(robot.OPEN_JAW);

        moveInches(0, 6, .5);

        robot.jaw.setPosition(robot.CLOSED_JAW);


    }
}
