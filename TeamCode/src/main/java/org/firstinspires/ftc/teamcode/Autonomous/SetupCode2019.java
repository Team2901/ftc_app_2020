package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.JewelFinder;
import org.firstinspires.ftc.robotcontroller.internal.LinearOpModeJewelCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Utility.BitmapUtilities;
import org.firstinspires.ftc.teamcode.Utility.FileUtilities;
import org.firstinspires.ftc.teamcode.Utility.RoverRuckusUtilities;
import org.firstinspires.ftc.teamcode.Utility.VuforiaUtilities;

import java.io.IOException;
import java.util.List;

import static org.firstinspires.ftc.robotcontroller.internal.JewelFinder.JEWEL_CONFIG_FILE_FORMAT;

@SuppressLint("DefaultLocale")
@Autonomous(name = "SetUpCode", group = "SetupCode")
public class SetupCode2019 extends LinearOpModeJewelCamera {
    private int leftHueTotal[] = {0, 0};
    private int middleHueTotal[] = {0, 0};
    private int rightHueTotal[] = {0, 0};

    private boolean useWebCam = true;
    private boolean useThreeJewels = true;
    private boolean postToJewels = true;

    private WebcamName webcam = null;
    private VuforiaLocalizer vuforia;

    public SetupCode2019() {}

    public SetupCode2019(boolean useWebCam, boolean useThreeJewels, boolean postToJewels) {
        this();
        this.useWebCam = useWebCam;
        this.useThreeJewels = useThreeJewels;
        this.postToJewels = postToJewels;
    }

    @Override
    public void waitForStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.addData("Webcam Attached", webcam != null && webcam.isAttached());
            telemetry.addData("Vuforia initialized", vuforia != null);
            telemetry.addData("Vuforia has camera", vuforia != null && vuforia.getCamera() != null);
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final FtcRobotControllerActivity activity = (FtcRobotControllerActivity) hardwareMap.appContext;
        final int cameraMonitorViewId = VuforiaUtilities.getCameraMonitorViewId(hardwareMap);

        // Setup webcam (if used)
        WebcamName webcam = null;
        if (useWebCam) {
            try {
                webcam = hardwareMap.get(WebcamName.class, "webcam");
            } catch (Exception e) {
                telemetry.addData("Warning", "webcam not found");
                telemetry.update();
            }
        }

        // Setup Vuforia
        final VuforiaLocalizer.Parameters parameters;
        if (useWebCam && webcam != null) {
            parameters = VuforiaUtilities.getWebCameraParameters(hardwareMap, webcam);
        } else {
            parameters = VuforiaUtilities.getBackCameraParameters(hardwareMap);
        }

        vuforia = VuforiaUtilities.getVuforia(parameters);

        // Create jewels on screen at locations read from config files
        readConfigFiles();
        activity.setupPreviewLayout(cameraMonitorViewId, this);

        // Wait until user is finished moving jewels around on screen
        waitForStart();

        // Save the jewels' on screen locations to config files
        saveConfigFiles();

        // Get the bitmap from vuforia
        final Bitmap bitmap = BitmapUtilities.getVuforiaImage(vuforia);

        telemetry.addData("Info", "It is safe to move the camera now");
        telemetry.update();

        try {
            final String fileName = "jewelBitmap.png";
            FileUtilities.writeBitmapFile(fileName, bitmap);
        } catch (Exception e) {
            telemetry.addData("Error writing to bitmap file", e.getMessage());
            telemetry.update();
        }

        // Tally hue totals for each jewel's location
        leftHueTotal = RoverRuckusUtilities.getJewelHueCount(bitmap, jewelLeft.getName(), this);
        middleHueTotal = RoverRuckusUtilities.getJewelHueCount(bitmap, jewelMiddle.getName(), this);
        rightHueTotal = RoverRuckusUtilities.getJewelHueCount(bitmap, jewelRight.getName(), this);

        // Determine jewel winner
        final BaseRoverRuckusAuto.GoldPosition winner;
        if (useThreeJewels) {
            winner = BitmapUtilities.findWinnerLocation(leftHueTotal, middleHueTotal, rightHueTotal);
        } else {
            //winner = BitmapUtilities.findCorrectGoldLocation(middleHueTotal, rightHueTotal);
            winner = BitmapUtilities.findWinnerLocation(middleHueTotal, rightHueTotal);
        }

        // Save jewel winner to file
        try {
            if (useThreeJewels) {
                FileUtilities.writeWinnerFile(winner, leftHueTotal, middleHueTotal, rightHueTotal);
            } else {
                FileUtilities.writeWinnerFile(winner, middleHueTotal, rightHueTotal);
            }
        } catch (IOException e) {
            telemetry.addData("Error writing to winner file", e.getMessage());
            telemetry.update();
        }

        // Update the jewels on screen with hue totals and winner/loser colors
        if (postToJewels) {
            postToJewels(winner);

            while (opModeIsActive()) {
                idle();
            }
        }

        activity.removeJewelFinder();
    }

    public void postToJewels(BaseRoverRuckusAuto.GoldPosition winner) {
        postToJewel(jewelLeft, leftHueTotal, BaseRoverRuckusAuto.GoldPosition.LEFT.equals(winner));
        postToJewel(jewelMiddle, middleHueTotal, BaseRoverRuckusAuto.GoldPosition.MIDDLE.equals(winner));
        postToJewel(jewelRight, rightHueTotal, BaseRoverRuckusAuto.GoldPosition.RIGHT.equals(winner));
    }

    public void postToJewel(final JewelFinder jewel,
                            final int hueTotal[],
                            final boolean isWinner) {
        jewel.post(new Runnable() {
            public void run() {
                jewel.setText(String.format("Y:%d,W:%d",hueTotal[0],hueTotal[1]));
                jewel.setBackgroundColor(isWinner ? Color.YELLOW : Color.WHITE);
            }
        });
    }

    public void readConfigFiles() {
        leftConfig = readConfigFile("Left");
        middleConfig = readConfigFile("Middle");
        rightConfig = readConfigFile("Right");
        telemetry.update();
    }

    public List<Integer> readConfigFile(String name) {
        String fileName = String.format(JEWEL_CONFIG_FILE_FORMAT, name);
        try {
            return FileUtilities.readIntegerConfigFile(fileName);
        } catch (IOException e){
            telemetry.addData("Error reading from " + fileName, e.getMessage());
            return null;
        }
    }

    public void saveConfigFiles() {
        saveConfigFile(jewelLeft);
        saveConfigFile(jewelMiddle);
        saveConfigFile(jewelRight);
        telemetry.update();
    }

    public void saveConfigFile(JewelFinder jewel) {
        String fileName = jewel.getConfigFileName();
        try {
            FileUtilities.writeConfigFile(fileName, jewel.getBoxPct());
        } catch (Exception e) {
            telemetry.addData("Error writing to " + fileName, e.getMessage());
        }
    }
}




