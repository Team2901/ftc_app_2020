package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import java.util.List;

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
        double fLeftStart = Math.abs(robot.frontLeft.getCurrentPosition());
        while (skyStoneCenterPercentDiff==null || Math.abs(skyStoneCenterPercentDiff ) > 10) {
            telemetry.addData("loop is running", "");
            telemetry.update();

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

        double fLeftEnd = Math.abs(robot.frontLeft.getCurrentPosition());
        double diff = Math.abs(fLeftEnd-fLeftStart);

        turnTo(90);

        robot.jaw.setPosition(robot.OPEN_JAW);
        this.moveForward(24);
        
        double currentAngle = robot.getAngle();
        double targetAngle = currentAngle + 90;

        while (robot.wheelsAreBusy()) {

        }

        /*
        1.) Move waffle to the middle position
            Move forward, grab, drag back
        2.) Move right
            To go underneath the bridge
        3.) Find skystone
            Identify if there are three stones, if not, go to next location
        4.) Get skystone
        5.) Move right forward direction
        6.) Turn 180 degrees clockwise
        7.) Place block
        8.) Place Capstone on block
        9.) Return to starting position
            Go left, forward, and then left, turn 180 degrees
        10.) Go to step 3
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

    public void goToAngle(double angleStart, double angleGoal) {

        robot.swerveTurn(0);

        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < 1 && opModeIsActive()) {

        }

        double angleCurrent = angleStart;

        while (Math.abs(angleGoal - angleCurrent) > GO_TO_ANGLE_BUFFER && opModeIsActive()) {
            angleCurrent = robot.getAngle();
            double power = getPower(angleCurrent, angleGoal, angleStart);
            robot.setWheelMotorPower(power, power, power, power);

            telemetry.addData("Start Angle ", "%.1f", angleStart);
            telemetry.addData("Goal Angle  ", "%.1f", angleGoal);
            telemetry.addData("Cur Angle   ", "%.1f", angleCurrent);
            telemetry.addData("Remain Angle", "%.1f", AngleUnit.normalizeDegrees(angleGoal - angleCurrent));
            telemetry.addData("Power       ", "%.2f", power);
            telemetry.update();
            idle();
        }
    }

    public double getPower(double absCurrent, double absGoal, double absStart) {
        double relCurrent = AngleUtilities.getNormalizedAngle(absCurrent - absStart);
        double relGoal = AngleUtilities.getNormalizedAngle(absGoal - absStart);
        double remainingDistance = AngleUtilities.getNormalizedAngle(relGoal - relCurrent);

        double basePower = 0.01 * remainingDistance;
        double stallPower = 0.1 * Math.signum(remainingDistance);
        return Range.clip(basePower + stallPower, -1, 1);
    }

}
