package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "BlueQuarry1StonePark", group = "")

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

public class BlueQuarry1StonePark extends BaseSkyStoneAuto {
    private static final String VUFORIA_KEY = "AYhwTMH/////AAABmR7oFvU9lEJTryl5O3jDSusAPmWSAx5CHlcB/" +
            "IUoT+t7S1pJqTo7n3OwM4f2vVULA0T1uZVl9i61kWldhVqxK2+kyBNI4Uld8cYgHaNIQFsL/NsyBrb3Zl+1ZFBR" +
            "tpI5BjPnJkivkDsGU0rAFd+vPkyZt0p3/Uz+50eEwMZrZh499IsfooWkGX1wobjOFeA7DYQU+5ulhc1Rdp4mqjj" +
            "uKrS24Eop0MKJ+PwvNJhnN4LqIWQSfSABmcw9ogaeEsCzJdowrpXAcSo9d+ykJFZuB92iKN16lC9dRG3PABt26o" +
            "lSUCeXJrC4g6bEldHlmTc51nRpix6i1sGfvNuxlATzuRf5dtX/YlQm2WvvG9TilHbz";

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        // Step 0) Initialize robot and web camera with TensorFlow
        robot.init(hardwareMap);


        initAndActivateWebCameraWithTensorFlow();
        robot.crane.setPosition(0);
        robot.wrist.setPosition(.5);

        // Step 0) Point wheels forward
        robot.swerveStraight(0, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for start
        waitForStart();

        this.moveInches(0, 2, .2);

        robot.swerveStraight(270, 0);
        robot.crane.setPosition(1);
        robot.wrist.setPosition(.5);
        double t = 0;
        while (t < 10) {
            t++;
        }
        // Save the robot's current position prior to search for a skystone
        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());

        // Step 1) Move forwards/backwards until a skystone location is within 10% of the center of the camera's view
//stuff
        Float skyStoneCenterPercentDiff = findSkyStone();

        while (skyStoneCenterPercentDiff == null || Math.abs(skyStoneCenterPercentDiff) > 3) {
            telemetry.addData("loop is running", "");
            telemetry.addData("percent dif.", skyStoneCenterPercentDiff);
            telemetry.update();

            if (skyStoneCenterPercentDiff == null) {
                // If we don't see a skystone: Move forwards
                robot.swerveStraight(270, 0.2);
            } else if (skyStoneCenterPercentDiff < 0) {
                // If the skystone is to the left: Move backwards
                robot.swerveStraight(270, 0.3);
            } else {
                // If the skystone is to the right: Move forwards
                robot.swerveStraight(270, -0.3);
            }

            // Update the skystone location
            skyStoneCenterPercentDiff = findSkyStone();
        }

        // Robot is now in front of a skystone, stop moving
        robot.swerveStraight(0, 0);

        telemetry.addData("out of loop", "");
        telemetry.addData("percent dif.", skyStoneCenterPercentDiff);
        telemetry.update();
        // Calculate in inches how far the robot has moved while finding the skystone
        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = Math.abs(fLeftEnd - fLeftStart);
        double diffInches = diff / robot.inchesToEncoder;


        // Step 2) Turn to face the skystone
        //turnTo(0, .2);

        // Step 3) Open the jaw
        robot.jaw.setPosition(robot.OPEN_JAW);
        robot.crane.setPosition(1);

        // Step 4) Move forwards 2 feet towards the skystone
        this.moveInches(0, 32, .2);

        // Step 5) Close the jaw on the skystone
        robot.jaw.setPosition(robot.CLOSED_JAW);


        this.moveInches(0, -5, .2);


        robot.swerveTurn(0);
        turnTo(90, .2);

        robot.swerveStraight(0, 0);

        this.moveInches(0, diffInches, .3);

        turnTo(90, .2);


        this.moveInches(0, 60, .4);

        robot.moveLift(50 );


        turnTo(0, .2);

        moveInches(0, 12, .3);

        robot.jaw.setPosition(robot.OPEN_JAW);

        turnTo(0, .2);

        moveInches(0, -12, .3);

        turnTo(-90, .2);

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
