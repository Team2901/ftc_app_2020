package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "CrawlToSkystone-Programmer", group = "new_programmer")

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
public class CrawlToSkystone extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        // Step 0) Initialize robot and web camera with TensorFlow
        robot.init(hardwareMap);
        initAndActivateWebCameraWithTensorFlow();

        // Step 0) Point wheels forward
        robot.swerveStraight(0, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Save the robot's current position prior to search for a skystone
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());

        // Step 1) Move forwards/backwards until a skystone location is within 10% of the center of the camera's view

        Float skyStoneCenterPercentDiff = findSkyStone();

        while (skyStoneCenterPercentDiff == null || Math.abs(skyStoneCenterPercentDiff) > 10) {
            telemetry.addData("loop is running", "");
            telemetry.update();

            if (skyStoneCenterPercentDiff == null) {
                // If we don't see a skystone: Move forwards
                robot.swerveStraight(0, 0.3);
            } else if (skyStoneCenterPercentDiff < 0) {
                // If the skystone is to the left: Move backwards
                robot.swerveStraight(0, -0.3);
            } else {
                // If the skystone is to the right: Move forwards
                robot.swerveStraight(0, 0.3);
            }

            // Update the skystone location
            skyStoneCenterPercentDiff = findSkyStone();
        }

        // Robot is now in front of a skystone, stop moving
        robot.swerveStraight(0, 0);

        telemetry.addData("out of loop", "");
        telemetry.update();

        // Calculate in inches how far the robot has moved while finding the skystone
        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = (fLeftEnd - fLeftStart);
        double diffInches = diff / robot.inchesToEncoder;

        // Step 2) Turn to face the skystone
        turnTo(90);

        // Step 3) Open the jaw
        robot.jaw.setPosition(robot.OPEN_JAW);

        // Step 4) Move forwards 2 feet towards the skystone
        this.moveInches(0, 24, .2);

        // Step 5) Close the jaw on the skystone
        robot.jaw.setPosition(robot.CLOSED_JAW);

        // Step 6) Move backwards 2 feet away from the skystone
        this.moveInches(0, -24, .2);

        // Step 7) Turn to face towards the building zone
        turnTo(90);

        // Step 8) Move back to where we were in step 1
        this.moveInches(0, -diffInches, .2);

        // Step 9) Move forwards to in front of the waffle
        // TODO

        // Step 10) deposit skystone on waffle
        // TODO

        // Step 11) Park under the skybridge
        // TODO

        while (opModeIsActive()) {
        }
    }
}
