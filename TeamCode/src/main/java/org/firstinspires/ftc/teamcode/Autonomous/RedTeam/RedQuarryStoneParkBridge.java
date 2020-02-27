package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous(name = "Red Quarry Stone Park Bridge", group = "_RED")
public class RedQuarryStoneParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        // Step 0) Initialize robot and web camera with TensorFlow
        init(false);

        robot.crane.setPosition(0);
        robot.wrist.setPosition(.5);
        robot.setGrabberPositition(.7, .84);

        // Wait for start
        waitForStart();

        // Move forwards 2 inches from the wall to turn wheels
        this.moveInches(0, 2, .2);

        // Extend crane
        robot.crane.setPosition(1);
        robot.wrist.setPosition(.5);

        // Step 3) Open the jaw
        robot.jaw.setPosition(robot.OPEN_JAW);

        // Step 4) Move forwards 2 feet towards the skystone
        this.moveInches(0, 32, .2);

        // Step 5) Close the jaw on the skystone
        robot.jaw.setPosition(robot.CLOSED_JAW);


        this.moveInches(0, -5, .2);


        robot.swerveTurn(0);
        turnTo(-90, .2);

        this.moveInches(0, 36, .4);

        robot.jaw.setPosition(robot.OPEN_JAW);


        while (opModeIsActive()) {
        }
    }


}
