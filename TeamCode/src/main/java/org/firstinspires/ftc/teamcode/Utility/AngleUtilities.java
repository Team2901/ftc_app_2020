package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AngleUtilities {

    /**
     * Normalizes an angle to be between -180 and 179 degrees
     *
     * @param angle angle to be normalized
     * @return an angle to be between -180 and 179 degrees
     */
    public static double getNormalizedAngle(double angle) {
        return AngleUnit.normalizeDegrees(angle);
    }

    public static double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;
    }

    /**
     * Normalizes an angle to be between 0 and 359 degrees
     *
     * @param angle angle to be normalized
     * @return an angle between 0 and 359 degrees
     */
    public static double getPositiveNormalizedAngle(double angle) {
        return (getNormalizedAngle(angle) + 360) % 360;
    }
}
