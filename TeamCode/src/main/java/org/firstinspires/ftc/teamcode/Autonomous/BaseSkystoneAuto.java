package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware.WHEEL_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware.WHEEL_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;


@Autonomous (name = "BaseAuto")
public class BaseSkystoneAuto extends LinearOpMode {

    public SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        robot.swerveStraight(0, 0);
    }

    public double getPower(double x, double y, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return AngleUtilities.getRadius(x,y);
        }
    }

    public void turnWheels(double goalAngle){
        robot.swerveMove(goalAngle,goalAngle,goalAngle,goalAngle, 0);
    }

    public void runToPosition( double goalPosition){

    }
}

