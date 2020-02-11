package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;


/**
 *  Steps
 *  0) initialize robot and web camera with TensorFlow, point wheels forward
 *  1) Move forwards/backwards until a skystone location is within 10% of the center of the camera's view
 *  2) Turn to face the skystone
 *  3) Open the jaw
 *  4) Move forwards 2 feet towards the skystone
 *  5) Close the jaw on the skystone
 *  6) Move backwards 2 feet away from the skystone
 *  7) Turn to face towards the building zone
 *  8) Move back to where we were in step 1
 *
 *  TODO:
 *  9) Move forwards to in front of the waffle
 *  10) deposit skystone on waffle
 *  11) Park under the skybridge
 */

@SuppressLint("DefaultLocale")
@Autonomous(name = "BlueQuarryMtoebes", group = "")
public class BlueQuarryMtoebes extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        // Step 0) Initialize robot and web camera with TensorFlow
        robot.init(hardwareMap);

        initAndActivateWebCameraWithTensorFlow();
        robot.crane.setPosition(0);
        robot.wrist.setPosition(.5);
        robot.setGrabberPositition(.7, .84);

        // Step 0) Point wheels forward
        robot.swerveStraight(0, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Move forwards 2 inches from the wall to turn wheels
        this.moveInches(0, 2, .2);

        // Make wheels point right
        robot.swerveStraight(-90, 0);

        // Extend crane
        robot.crane.setPosition(1);

        // Save the robot's current position prior to search for a skystone
        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Step 1) Move forwards/backwards until a skystone location is within 10% of the center of the camera's view
        Float skyStoneCenterPercentDiff = findSkyStone();

        int location;

        if (skyStoneCenterPercentDiff != null && skyStoneCenterPercentDiff < -30) {
            location = -1;
        } else if (skyStoneCenterPercentDiff != null && skyStoneCenterPercentDiff > 30) {
             location = 0;
        } else {
            location = 1;
        }

        telemetry.speak(skyStoneCenterPercentDiff != null ? skyStoneCenterPercentDiff.toString() : "Unknown");
        telemetry.speak(String.format("diff %s location %d", skyStoneCenterPercentDiff != null ? skyStoneCenterPercentDiff.toString() : "Unknown", location));
        telemetry.addData("skyStoneCenterPercentDiff", skyStoneCenterPercentDiff);

        final double cameraOffset = 4;

        final double skystoneOffset = location * 6 - cameraOffset;

        this.moveInches(90, skystoneOffset, .2);

        this.wait(1000);

        // Step 3) Open the jaw
        robot.jaw.setPosition(robot.OPEN_JAW);
        robot.crane.setPosition(1);

        // Step 4) Move forwards 2 feet towards the skystone
        this.moveInches(0, 28, .2);

        // Step 5) Close the jaw on the skystone
        robot.jaw.setPosition(robot.CLOSED_JAW);

        this.wait(100);

        this.moveInches(0, -20, .5);

        this.wait(1000);

        robot.swerveStraight(-90, 0);

        // Move to waffle
        double waffleDistance = (3 * 24 + 9);
        this.moveInches(-90, waffleDistance, .5);

        this.wait(1000);

        // Raise lift (lift is semi broke atm, cant do run to position)
        //moveLift(500, 1000);

        // Move towards waffle
        this.moveInches(0, 20, .5);

        // Lower lift (lift is semi broke atm, cant do run to position)
        //moveLift(-500, 1000);

        // Backup with waffle
        backupInArc(90, .1);

        this.wait(1000);

        // Release skystone
        robot.jaw.setPosition(robot.OPEN_JAW);

        this.wait(1000);

        moveInches(90, 10, .5);

        this.wait(1000);

        // Park
        moveInches(90, -24, .5);

        while (opModeIsActive()) {
        }
    }


    public void moveLift(int ticks, int milisecondTimeout) {

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(ticks);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.25);

        ElapsedTime liftTimer = new ElapsedTime();

        while (robot.lift.isBusy() && liftTimer.milliseconds() < milisecondTimeout && opModeIsActive()) {

        }

        robot.lift.setPower(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backupInArc(double endAngle, double power) {

        final ElapsedTime arcTimer = new ElapsedTime();

        final double startAngle = robot.getAngle();
        while (robot.getAngle() < endAngle) {
            double currentAngle = startAngle + (arcTimer.milliseconds() * 33);
            robot.swerveStraight(currentAngle, -power);

            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("", robot.frontLeftSwerveWheel.toString());
            telemetry.addData("", robot.frontRightSwerveWheel.toString());
            telemetry.addData("", robot.backLeftSwerveWheel.toString());
            telemetry.addData("", robot.backRightSwerveWheel.toString());
            telemetry.update();
        }

        robot.swerveStraight(endAngle, 0);
    }
}
