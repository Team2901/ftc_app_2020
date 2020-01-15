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

public class BaseCamera {

    protected int cameraMonitorViewId;
    protected int tfodMonitorViewId;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaLocalizer vuforia;

    protected VuforiaLocalizer initBackCamera(final HardwareMap hardwareMap,
                                           final boolean showView) {

        final VuforiaLocalizer.Parameters parameters = VuforiaUtilities.getBackCameraParameters(hardwareMap, showView);
        return init(hardwareMap, parameters);
    }

    protected VuforiaLocalizer initWebCamera(final HardwareMap hardwareMap,
                                          final String configName,
                                          final boolean showView) {

        final WebcamName webcamName = hardwareMap.get(WebcamName.class, configName);
        final VuforiaLocalizer.Parameters parameters = VuforiaUtilities.getWebCameraParameters(hardwareMap, webcamName, showView);
        return init(hardwareMap, parameters);
    }


    private VuforiaLocalizer init(final HardwareMap hardwareMap,
                      final VuforiaLocalizer.Parameters parameters) {

        if (vuforia == null) {
            this.parameters = parameters;
            this.cameraMonitorViewId = VuforiaUtilities.getCameraMonitorViewId(hardwareMap);
            this.tfodMonitorViewId = VuforiaUtilities.getTfodMonitorViewId(hardwareMap);
            this.vuforia = VuforiaUtilities.getVuforia(parameters);
        }

        return this.vuforia;
    }
}
