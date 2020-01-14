package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

@SuppressLint("DefaultLocale")
@TeleOp(name = "ToolBox", group = "competition")
public class ToolBox extends LinearOpMode {

    public BaseSkyStoneHardware robot = null;
    public static final int GO_TO_ANGLE_BUFFER = 5;

    public ToolBox(BaseSkyStoneHardware robot){

        this.robot = robot;

    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);

        waitForStart();

        while (opModeIsActive()) {

        }
    }

    public void turnTo (double angle){

        goToAngle(robot.getAngle(), angle);
    }

    public void moveInches (double angle, double inches, double power){

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setWheelTargetPositions((int)(-inches*robot.inchesToEncoder));

        robot.swerveStraight(angle,power);

        while (robot.wheelsAreBusy() && opModeIsActive());

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.swerveStraight( angle,0);
    }

    public void goToAngle(double angleStart, double angleGoal) {

        double angleCurrent = angleStart;

        while ((Math.abs(angleGoal - angleCurrent) > GO_TO_ANGLE_BUFFER) && opModeIsActive()) {
            angleCurrent = robot.getAngle();
            double power = getPower(angleCurrent, angleGoal, angleStart);
            robot.swerveTurn(power);

            telemetry.addData("Start Angle ", "%.1f", angleStart);
            telemetry.addData("Goal Angle  ", "%.1f", angleGoal);
            telemetry.addData("Cur Angle   ", "%.1f", angleCurrent);
            telemetry.addData("Remain Angle", "%.1f", AngleUnit.normalizeDegrees(angleGoal - angleCurrent));
            telemetry.addData("Power       ", "%.2f", power);
            telemetry.update();

        }
        robot.swerveTurn(0);
        telemetry.addData("Is Stopped" , "");
        telemetry.update();
    }

    public double getPower(double absCurrent, double absGoal, double absStart) {
        double relCurrent = AngleUtilities.getNormalizedAngle(absCurrent - absStart);
        double relGoal = AngleUtilities.getNormalizedAngle(absGoal - absStart);
        double remainingDistance = AngleUtilities.getNormalizedAngle(relGoal - relCurrent);

        double basePower = 0.045 * remainingDistance;
        double stallPower = 0.1 * Math.signum(remainingDistance);
        return Range.clip(basePower + stallPower, -1, 1);
    }

    public void platformParkInner(int team){
        double colorAngle;
        if (team == Color.RED){
            colorAngle = robot.ROBOT_LEFT_ANGLE;
        }
        else {
            colorAngle = robot.ROBOT_RIGHT_ANGLE;

        }
        //Step one: turn wheels 90 degrees counterclockwise and go forward 28.5 inches and lower grabbers.
        moveInches(colorAngle,28.5 , 0.5 );
        robot.rightGrabber.setPosition (robot.GRABBER_MAX);
        robot.leftGrabber.setPosition(robot.GRABBER_MAX);
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds()< 500);

        //Step two: back up 26 inches and raise grabbers.
        moveInches(colorAngle,-26 , 0.5);
        robot.rightGrabber.setPosition (robot.GRABBER_MIN);
        robot.leftGrabber.setPosition(robot.GRABBER_MIN);
        //Step three: turn wheels 90 degrees counterclockwise and slide out 2 ft.
        moveInches(robot.ROBOT_FRONT_ANGLE, -24  , 0.5);
        //Step four: turn wheels to position 90, move forwards 2 ft
        moveInches(colorAngle, 24, 0.5);
        //Step Five: Turn Wheels to 0, move forward 2 ft
        moveInches(robot.ROBOT_FRONT_ANGLE, -24, 0.5);
        //Step  Six: Stop
    }
}

