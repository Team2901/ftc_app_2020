package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;
import org.firstinspires.ftc.teamcode.Utility.VuforiaUtilities;

import static org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware.WEB_CAM_NAME;

//@Disabled
@Autonomous(name = "CrawlToSkystone-Programmer", group = "new_programmer")

public class CrawlToSkystone extends BaseSkyStoneAuto {


    private Float skyStoneCenterPercentDiff;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        /*
        1.) Find skystone
        2.) Get skystone
            Move forward, activate intake
        3.) Move right forward direction
        4.) Turn 90 degrees counterclockwise
            Find the waffle using the camera
        5.) If the waffle is not in the corner, put it in the corner
        6.) If the waffle is already in the corner
            Move to waffle, place block
         7.) Place Capstone on block
         */
        // Step 1 Find Skystone

        robot.initWebCamera(hardwareMap);

        robot.webCamera.activateTfod();

        if (robot.webCamera.hasError()) {
            telemetry.addData("FAILED!",  robot.webCamera.errorMessage);

        } else {
            telemetry.addData("Successful!", "");
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, WEB_CAM_NAME);
        final VuforiaLocalizer.Parameters parameters = VuforiaUtilities.getWebCameraParameters(hardwareMap, webcamName, true);
        robot.webCamera.init(hardwareMap, parameters);

        skyStoneCenterPercentDiff = findSkyStone();

        // skyStoneGridLocation = convertPositionToGridOffset(skyStoneCenterPercentDiff);


        //Step 2 Get Skystone

        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //geting position before going to skystone
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());
        while (skyStoneCenterPercentDiff==null || Math.abs(skyStoneCenterPercentDiff ) > 10) {
            telemetry.addData("loop is running", "");
            telemetry.update();
//crawling to find skystone
            if (skyStoneCenterPercentDiff==null)
            {
                robot.swerveStraight(0, 0.3);
            }
            else if(skyStoneCenterPercentDiff<0)
            {
                robot.swerveStraight(0,-.3);
            }
            else
            {
                robot.swerveStraight(0, .3);
            }

            skyStoneCenterPercentDiff=findSkyStone();

        }


        telemetry.addData("out of loop", "");
        telemetry.update();

        robot.swerveStraight(0,0);
//fLeftEnd is were we end
        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = (fLeftEnd-fLeftStart);
        double diffInches = diff / robot.inchesToEncoder;
//we turn then open jaw then move forward
        /*3*/ turnTo(90);

        /*4*/ robot.jaw.setPosition(robot.OPEN_JAW);
        /*5*/ this.moveInches(90, 24, .2);
        /*6*/ robot.jaw.setPosition(robot.CLOSED_JAW);
        /*7*/ this.moveInches(90, -24, .2);

        /*8*/ turnTo(-90);
        /*9*/ this.moveInches(0, -diffInches, .2);

        /*
        1.) find current position
        2.) crawl to skystone
        3.) turn 90
        4.) open jaw
        5.) Move right forward 24''
        6.) close jaw
        7.) move back 24''
        8.) turn -90
        9.) move back how ever much we went forward (useing diff on line 121)

         */
        while (opModeIsActive()) {
        }
    }

}
