package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;



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

@Autonomous(name = "RedQuarrySkyStoneParkBridge", group = "")
public class RedQuarrySkyStoneParkBridge extends BaseSkyStoneAuto {

    final static int CONFIDENCE_PERCENTAGE = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        // Step 0) Initialize robot and web camera with TensorFlow
        robot.init(hardwareMap);


        initAndActivateWebCameraWithTensorFlow();
        //robot.crane.setPosition(0);
        robot.wrist.setPosition(.5);
        robot.setGrabberPositition(.7,.84);

        // Step 0) Point wheels forward
        robot.swerveStraightAbsolute(0, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        robot.opMode = this;
        // Wait for start
        waitForStart();

        this.moveInchesAbsolute(0, 5, .2);

        robot.swerveStraightAbsolute(90, 0);
        //robot.crane.setPosition(1);
        //robot.wrist.setPosition(.5);
        /*double t = 0;
        while (t < 10) {
            t++;
        }*/
        // Save the robot's current position prior to search for a skystone
        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());

        // Step 1) Move forwards/backwards until a skystone location is within 10% of the center of the camera's view
        Float skyStoneCenterPercentDiff = findSkyStone();
        Float skyStoneOffsetPercentDiff = skyStoneCenterPercentDiff == null ? null : skyStoneCenterPercentDiff + 45;

        while (skyStoneOffsetPercentDiff == null /* don't see skystone yet */
                || Math.abs(skyStoneOffsetPercentDiff) > CONFIDENCE_PERCENTAGE /* overshot or undershot */) {
            telemetry.addData("loop is running", "");
            telemetry.addData("percent dif.", skyStoneCenterPercentDiff);
            telemetry.addData("percent offset", skyStoneOffsetPercentDiff);
            telemetry.update();

            if (skyStoneOffsetPercentDiff == null) {
                // If we don't see a skystone: Move forwards
                robot.swerveStraightAbsolute(90, 0.2);
            } else if (skyStoneOffsetPercentDiff < 0) {
                // If the skystone is to the left: Move backwards
                robot.swerveStraightAbsolute(90, 0.3);
            } else {
                // If the skystone is to the right: Move forwards
                robot.swerveStraightAbsolute(90, -0.3);
            }

            // Update the skystone location
            skyStoneCenterPercentDiff = findSkyStone();
            skyStoneOffsetPercentDiff = skyStoneCenterPercentDiff == null ? null : skyStoneCenterPercentDiff + 45;
        }

        robot.swerveStraightAbsolute(0, 0);
        if (Math.abs(robot.getAngle()) > 5) {
            robot.wait(1000, this);
            turnTo(0);
            robot.wait(1000, this);
        }
        telemetry.addData("out of loop", "");
        telemetry.addData("percent dif.", skyStoneCenterPercentDiff);
        telemetry.addData("percent offset", skyStoneOffsetPercentDiff);
        telemetry.update();
        // Calculate in inches how far the robot has moved while finding the skystone
        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = Math.abs(fLeftEnd - fLeftStart);
        double diffInches = diff / robot.inchesToEncoder;


        // Step 2) Turn to face the skystone
        //turnTo(0, .2);

        // Step 3) Open the jaw
        robot.jaw.setPosition(robot.OPEN_JAW);
        //robot.crane.setPosition(1);

        // Step 4) Move forwards 2 feet towards the skystone
        this.moveInchesAbsolute(0, 32, .2);

        // Step 5) Close the jaw on the skystone
        robot.jaw.setPosition(robot.CLOSED_JAW);

        //back up so we don't hit bridge
        this.moveInchesAbsolute(0, -15, .2);

        if (Math.abs(robot.getAngle()) > 5) {
            robot.wait(1000, this);
            turnTo(0);
            robot.wait(1000, this);
        }
        this.moveInchesAbsolute(270, diffInches, .3);

        if (Math.abs(robot.getAngle()) > 5) {
            robot.wait(1000, this);
            turnTo(0);
            robot.wait(1000, this);
        }
        this.moveInchesAbsolute(270, 72, .3);

        //robot.moveLift(50 );

        moveInchesAbsolute(0, 28, .3);

        robot.jaw.setPosition(robot.OPEN_JAW);

        moveInchesAbsolute(0, -22, .3);

        moveInchesAbsolute(90, 40, .3);
/*
        // Step 8) Move back to where we were in step 1


        // Step 9) Move forwards to in front of the waffle
        // TODO

        // Step 10) deposit skystone on waffle
        // TODO

        // Step 11) Park under the skybridge
        // TODO
*/
        while (opModeIsActive()) {
        }
    }


}
