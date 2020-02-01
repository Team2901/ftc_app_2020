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

//@Disabled
@Autonomous(name = "CrawlToSkystone", group = "new_programmer")

public class CrawlToSkystone extends BaseSkyStoneAuto {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_BUTTER = "Stone";
    private static final String LABEL_SKY_BUTTER = "Skystone";
    private Float skyStoneCenterPercentDiff;
    private double skyStoneGridLocation;
    private static final double BLOCK_OFFSET = 30.0;
    public static final int LEFT_POSITION = 33;
    public static final int RIGHT_POSITION = 66;
    public static final int CENTER_POSITION = 50;

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();

    private static final String VUFORIA_KEY = "AYhwTMH/////AAABmR7oFvU9lEJTryl5O3jDSusAPmWSAx5CHlcB/" +
            "IUoT+t7S1pJqTo7n3OwM4f2vVULA0T1uZVl9i61kWldhVqxK2+kyBNI4Uld8cYgHaNIQFsL/NsyBrb3Zl+1ZFBR" +
            "tpI5BjPnJkivkDsGU0rAFd+vPkyZt0p3/Uz+50eEwMZrZh499IsfooWkGX1wobjOFeA7DYQU+5ulhc1Rdp4mqjj" +
            "uKrS24Eop0MKJ+PwvNJhnN4LqIWQSfSABmcw9ogaeEsCzJdowrpXAcSo9d+ykJFZuB92iKN16lC9dRG3PABt26o" +
            "lSUCeXJrC4g6bEldHlmTc51nRpix6i1sGfvNuxlATzuRf5dtX/YlQm2WvvG9TilHbz";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

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

        initVuforia();
// intializing vuforia and tenser flow
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        } else {
            telemetry.addData("We Regret to Inform You", "Tensor flow has failed to initialize");
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

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
        double diff = Math.abs(fLeftEnd-fLeftStart);
//we turn then open jaw then move forward
        /*3*/ turnTo(90);

        /*4*/ robot.jaw.setPosition(robot.OPEN_JAW);
        /*5*/ this.moveForward(24);
        /*6*/ robot.jaw.setPosition(robot.CLOSED_JAW);
        /*7*/ this.moveForward(-24);

        /*8*/ turnTo(-90);
        /*9*/ this.moveForward(diff);

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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_BUTTER, LABEL_SKY_BUTTER);
    }



    private double convertPositionToGridOffset(int position) {
        return 54.4;
    }

    private void moveForward(double inches) {
        telemetry.addData("Moved to a position in front of oneself", inches);
    }

    private void moveLeft(double inches) {
        telemetry.addData("Moved to a position to the left of oneself", inches);
    }

    private void moveRight(double inches) {
        telemetry.addData("Moved to a position to the right of oneself", inches);
    }

    public static final int GO_TO_ANGLE_BUFFER = 3;



    public double getCurrentTurnPower(double absCurrent, double absGoal, double absStart, double maxPower) {
        double relCurrent = AngleUtilities.getNormalizedAngle(absCurrent - absStart);
        double relGoal = AngleUtilities.getNormalizedAngle(absGoal - absStart);
        double remainingDistance = AngleUtilities.getNormalizedAngle(relGoal - relCurrent);

        double basePower = 0.01 * remainingDistance;
        double stallPower = 0.1 * Math.signum(remainingDistance);
        return Range.clip(basePower + stallPower, -1, 1);
    }

}
