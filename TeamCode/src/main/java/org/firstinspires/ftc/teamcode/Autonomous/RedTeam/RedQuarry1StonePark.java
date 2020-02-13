package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
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

@Autonomous(name = "RedQuarry1StonePark", group = "")
public class RedQuarry1StonePark extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot starts on stone side right next to the skybridge

        // Step 0) Initialize robot and web camera with TensorFlow
        robot.init(hardwareMap);
        initAndActivateWebCameraWithTensorFlow();


        robot.crane.setPosition(0);
        robot.wrist.setPosition(.5);
        robot.setGrabberPositition(.7,.84);

        // Step 0) Point wheels forward
        robot.swerveStraight(0, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Move forwards 2 inches from the wall to turn wheels
        this.moveInches(0, 2, .2);

        // Make wheels point right toward audience wall
        robot.swerveStraight(90, 0);


        robot.crane.setPosition(1);
        robot.wrist.setPosition(.5);

        /*  ?????????
        // what is this .. should be a wait loop for servos to reach target ?
        //??????
        double t = 0;
        while (t < 10) {
            t++;
        }
        */


        // Save the robot's current position prior to search for a skystone
        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //??????????????
        // looks like a cut-n-pasteof SkystoneScanner()
        // with enhancements... can we update SkystoneScanner() ?
        // and make the angle correct based on red or blue
        //??????????????
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());

        // Step 1) Move forwards/backwards until a skystone location is within x% of the center of the camera's view
        Float skyStoneCenterPercentDiff = findSkyStone();

        //?????????????????
        // need to verify the camera can not see 2 skystones at once
        // or it might occillate between trying to go to one vs the other
        // If you can see more than 1, then move forward ?
        // is this a practical problem
        //???????????????
        while (
                // sky stone has not come into view or there was an error initializing Tensor flow
                skyStoneCenterPercentDiff == null
                ||  // OR
                // robot webcam sees skystone, but is not within 3% of the skystone's center point
                Math.abs(skyStoneCenterPercentDiff) > 3) {

            telemetry.addData("loop is running", "");
            telemetry.addData("percent dif.", skyStoneCenterPercentDiff);
            telemetry.update();

            if (skyStoneCenterPercentDiff == null) {
                // If we don't see a skystone: Move forwards...
                //??????????
                // until we hit the audience wall ????
                // maybe we should check the encoder count and stop before then
                //?????????
                robot.swerveStraight(90, 0.2);

            } else if (skyStoneCenterPercentDiff < 0) {
                // If the skystone is to the left: Move backwards (positive is left/backwards)
                robot.swerveStraight(90, 0.3);

            } else {
                // If the skystone is to the right: Move forwards (negative is right/forward)
                robot.swerveStraight(90, -0.3);
            }

            // Update the skystone location
            //???????
            // this call takes time.. and your motors are still running..
            // suggest not making this call if skystone has been found.
            skyStoneCenterPercentDiff = findSkyStone();

        }

        // Robot moves right 6 inches to be centered on the stone
        this.moveInches(0, 6, -0.3);

        // Robot webcam is now in front of a skystone, stop moving
        robot.swerveStraight(0, 0);

        telemetry.addData("out of loop", "");
        telemetry.addData("percent dif.", skyStoneCenterPercentDiff);
        telemetry.update();


        // Calculate in inches how far the robot has moved while finding the skystone
        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = Math.abs(fLeftEnd - fLeftStart);
        double diffInches = diff / robot.inchesToEncoder;

        // Step 3) Open the jaw
        robot.jaw.setPosition(robot.OPEN_JAW);
        robot.crane.setPosition(1);

        // Step 4) Move forwards 2 feet towards the skystone
        this.moveInches(0, 32, .2);

        // Step 5) Close the jaw on the skystone
        robot.jaw.setPosition(robot.CLOSED_JAW);

        // Step 6) move back from the stones a little bit so as not to get hung up on the
        // skystone bridge when passing under it
        this.moveInches(0, -5, .2);

        // Step 7) Turn the robot to face the back platform wall
        robot.swerveTurn(0);
        turnTo(270, .2);

        // Step 8) Return to starting position, but now facing back wall
        robot.swerveStraight(0, 0);
        this.moveInches(0, diffInches, .3);

        // Step 9) Continue moving under the skystone bridge (2.5 tiles)
        // assumes the platform has not been moved yet
        // we should be at the center of the platform
        turnTo(270, .2);
        this.moveInches(0, 60, .4);

        //robot.moveLift(50 );

        // Step 10) turn to face the platform, then Move forwards to in front of the waffle
        turnTo(0, .2);
        moveInches(0, 12, .3);

        // Step 12) release the skystone
        robot.jaw.setPosition(robot.OPEN_JAW);

        // Step 13) still facing the platform, back up .5 tiles
        turnTo(0, .2);
        moveInches(0, -12, .3);

        // Step 14) move 1.5 tiles to under the bridge
        // ?????
        // is 1.5 tiles enough
        // ?????
        turnTo(90, .2);
        moveInches(0, 40, .3);
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
