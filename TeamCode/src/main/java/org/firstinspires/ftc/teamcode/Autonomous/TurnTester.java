package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

public class TurnTester extends LinearOpMode {

    public final SkystoneHardware robot = new SkystoneHardware();

   /* public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = AngleUtilities.radiansDegreesTranslation(wheelAngleRad) - 90;
        double wheelAngleStandarized = AngleUtilities.getPositiveNormalizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }
    
    */

    /*public void swerveTurn(double joyStickRightPosX) {

        //Math mod????????
        double hardCodeThis = Math.sqrt(2) / 2;
        double fLAngle = joystickPositionToWheelAngle(-hardCodeThis, -hardCodeThis);
        double fRAngle = joystickPositionToWheelAngle(-hardCodeThis, hardCodeThis);
        double bLAngle = joystickPositionToWheelAngle(hardCodeThis, -hardCodeThis);
        double bRAngle = joystickPositionToWheelAngle(hardCodeThis, hardCodeThis);

        //swerveMove(fLAngle, fRAngle, bLAngle, bRAngle, joyStickRightPosX);
    }

     */


    @Override
    public void runOpMode() throws InterruptedException {

        // init hardware
        robot.init(this.hardwareMap);
        // move wheels to turn




        // wait for start
        this.waitForStart();
        // goto angle
        double currentAngle = robot.getAngle();
        this.goToAngle(currentAngle,currentAngle + 90);

    }

    public static final int GO_TO_ANGLE_BUFFER = 3;

        public void goToAngle ( double angleStart, double angleGoal){

            double angleCurrent = angleStart;

            while (Math.abs(angleGoal - angleCurrent) > GO_TO_ANGLE_BUFFER && opModeIsActive()) {
                angleCurrent = robot.getAngle();
                double power = getPower(angleCurrent, angleGoal, angleStart);
                robot.turn(-power);

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

