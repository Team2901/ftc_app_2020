package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuforiaUtilities {

    public static final int PIXEL_FORMAT_RGB565 = 1;
    public static final double MM_TO_INCHES = 0.0393701;
    public static final double INCHES_TO_MM = 25.4;
    public static final double FIELD_RADIUS = 1828.8;


    public static OpenGLMatrix phoneLocation = getMatrix(90, -0, -90,
            (int) (-6 * INCHES_TO_MM), (int) (1 * INCHES_TO_MM), (int) (18 * INCHES_TO_MM));

    public final static String VUFORIA_KEY = "AYhwTMH/////AAABmR7oFvU9lEJTryl5O3jDSusAPmWSAx5CHlcB/" +
            "IUoT+t7S1pJqTo7n3OwM4f2vVULA0T1uZVl9i61kWldhVqxK2+kyBNI4Uld8cYgHaNIQFsL/NsyBrb3Zl+1ZFBR" +
            "tpI5BjPnJkivkDsGU0rAFd+vPkyZt0p3/Uz+50eEwMZrZh499IsfooWkGX1wobjOFeA7DYQU+5ulhc1Rdp4mqjj" +
            "uKrS24Eop0MKJ+PwvNJhnN4LqIWQSfSABmcw9ogaeEsCzJdowrpXAcSo9d+ykJFZuB92iKN16lC9dRG3PABt26o" +
            "lSUCeXJrC4g6bEldHlmTc51nRpix6i1sGfvNuxlATzuRf5dtX/YlQm2WvvG9TilHbz";

    public static VuforiaLocalizer.Parameters getBackCameraParameters(HardwareMap hardwareMap) {
        return getBackCameraParameters(hardwareMap, true);
    }

    public static VuforiaLocalizer.Parameters getBackCameraParameters(HardwareMap hardwareMap,
                                                                      boolean withView) {
        VuforiaLocalizer.Parameters parameters = getParameters(hardwareMap, withView);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        return parameters;
    }

    public static VuforiaLocalizer.Parameters getWebCameraParameters(HardwareMap hardwareMap,
                                                                     WebcamName webcamName) {
        return getWebCameraParameters(hardwareMap, webcamName, true);
    }

    public static VuforiaLocalizer.Parameters getWebCameraParameters(HardwareMap hardwareMap,
                                                                     WebcamName webcamName,
                                                                     boolean withView) {
        VuforiaLocalizer.Parameters parameters = getParameters(hardwareMap, withView);
        parameters.cameraName = webcamName;
        return parameters;
    }

    public static VuforiaLocalizer.Parameters getParameters(HardwareMap hardwareMap,
                                                            boolean withView) {
        VuforiaLocalizer.Parameters parameters;
        if (withView) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id",
                            hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        return parameters;
    }

    public static VuforiaLocalizer getVuforia(VuforiaLocalizer.Parameters parameters) {
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
        return vuforia;
    }

    public static VuforiaTrackables setUpTrackables(VuforiaLocalizer vuforia,
                                                    VuforiaLocalizer.Parameters parameters) {

        VuforiaTrackables roverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blue = roverRuckus.get(0);
        VuforiaTrackable red = roverRuckus.get(1);
        VuforiaTrackable front = roverRuckus.get(2);
        VuforiaTrackable back = roverRuckus.get(3);

        blue.setName("blue");
        red.setName("red");
        front.setName("front");
        back.setName("back");

        OpenGLMatrix blueTrackablePosition = getMatrix(90, 0, -90, (float) FIELD_RADIUS, 0, (float) 152.4);
        OpenGLMatrix frontTrackablePosition = getMatrix(90, 0, 0, 0, (float) FIELD_RADIUS, (float) 152.4);
        OpenGLMatrix redTrackablePosition = getMatrix(90, 0, 90, (float) -FIELD_RADIUS, 0, (float) 152.4);
        OpenGLMatrix backTrackablePosition = getMatrix(90, 0, 180, 0, (float) -FIELD_RADIUS, (float) 152.4);

        blue.setLocation(blueTrackablePosition);
        red.setLocation(redTrackablePosition);
        front.setLocation(frontTrackablePosition);
        back.setLocation(backTrackablePosition);

        ((VuforiaTrackableDefaultListener) blue.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) red.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) front.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) back.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);

        return roverRuckus;
    }

    public static OpenGLMatrix getMatrix(float ax, float ay, float az, float dx, float dy, float dz) {

        return OpenGLMatrix.translation(dx, dy, dz).multiplied
                (Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                        AxesOrder.XYZ, AngleUnit.DEGREES, ax, ay, az));
    }

    public static OpenGLMatrix getLocation(VuforiaTrackables roverRuckus) {
        return getLocation(roverRuckus.get(0), roverRuckus.get(1), roverRuckus.get(2), roverRuckus.get(3));
    }

    public static OpenGLMatrix getLocation(VuforiaTrackable blue, VuforiaTrackable red,
                                           VuforiaTrackable front, VuforiaTrackable back) {
        OpenGLMatrix location = null;
        OpenGLMatrix blueLocation = ((VuforiaTrackableDefaultListener)
                blue.getListener()).getUpdatedRobotLocation();
        OpenGLMatrix redLocation = ((VuforiaTrackableDefaultListener)
                red.getListener()).getUpdatedRobotLocation();
        OpenGLMatrix backLocation = ((VuforiaTrackableDefaultListener)
                back.getListener()).getUpdatedRobotLocation();
        OpenGLMatrix frontLocation = ((VuforiaTrackableDefaultListener)
                front.getListener()).getUpdatedRobotLocation();

        if (blueLocation != null) {
            location = blueLocation;
        } else if (redLocation != null) {
            location = redLocation;
        } else if (backLocation != null) {
            location = backLocation;
        } else if (frontLocation != null) {
            location = frontLocation;
        }

        return location;
    }

    public static int getCameraMonitorViewId(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }
}