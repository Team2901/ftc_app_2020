package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Utility.VuforiaUtilities;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Utility.VuforiaUtilities.INCHES_TO_MM;

public class TensorFlowCamera extends BaseCamera {

    public TFObjectDetector tfod;

    public TFObjectDetector initBackCamera(final HardwareMap hardwareMap,
                                           final double minimumConfidence,
                                           final String assetName,
                                           final String... labels) {

        super.initBackCamera(hardwareMap, false);
        return initTfod(minimumConfidence, assetName, labels);
    }

    public TFObjectDetector initWebCamera(final HardwareMap hardwareMap,
                              final String configName,
                                          final double minimumConfidence,
                                          final String assetName,
                                          final String... labels) {

        super.initWebCamera(hardwareMap, configName,false);
        return initTfod(minimumConfidence, assetName, labels);
    }

    private TFObjectDetector initTfod(final double minimumConfidence,
                          final String assetName,
                          final String... labels) {

        if (tfod == null && vuforia != null) {
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = minimumConfidence;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(assetName, labels);
        }

        return tfod;
    }

    public void activateTfod() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void deactivateTfod() {
        if (tfod != null) {
            tfod.deactivate();
        }
    }

    public void shutdownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
