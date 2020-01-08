package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;

@SuppressLint("DefaultLocale")
@TeleOp(name = "ToolBox", group = "competition")
public class ToolBox extends LinearOpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public static final int GO_TO_ANGLE_BUFFER = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        swerveStraight(0, 0);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a){
                turnTo(90);
                while (robot.frontLeft.isBusy() && opModeIsActive());
            }else if (gamepad1.b){
                moveInches(45 ,24, 1);
            }else if (gamepad1.x){
                moveInches(0 ,24, 1);
            }else if (gamepad1.y){
                moveInches(90 ,24, 1);
            }
        }
    }
/*
    @Override
    public void init() {
        robot.init(hardwareMap);
        swerveStraight(0, 0);
    }

    @Override
    public void loop()  {

        if(gamepad1.a){
            turnTo(90);
            while (robot.frontLeft.isBusy());
        }else if (gamepad1.b){
            moveInches(45 ,24, .7);
        }else if (gamepad1.x){
            moveInches(0 ,24, .7);
        }else if (gamepad1.y){
            moveInches(90 ,24, .7);
        }
    }

    */

    public void turnTo (double angle){

        goToAngle(robot.getAngle(), angle);

    }

    public void moveInches (double angle, double inches, double power){

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setWheelTargetPositions((int)(-inches*robot.INCHES_TO_ENCODER));

        swerveStraight(angle,power);

        while (robot.wheelsAreBusy() && opModeIsActive());

        robot.setWheelMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        swerveStraight( angle,0);


    }

    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = AngleUtilities.radiansDegreesTranslation(wheelAngleRad) - 90;
        return AngleUtilities.getPositiveNormalizedAngle(wheelAngle);
    }

    public double getPower(double x, double y, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return AngleUtilities.getRadius(x,y);
        }
    }

    public void swerveStraight(double joyWheelAngle, double power) {
        swerveMove(joyWheelAngle, joyWheelAngle, joyWheelAngle, joyWheelAngle, power);
    }

    public void swerveTurn(double power) {

        double fLAngle = joystickPositionToWheelAngle(-1, -1);
        double fRAngle = joystickPositionToWheelAngle(-1, 1);
        double bLAngle = joystickPositionToWheelAngle(1, -1);
        double bRAngle = joystickPositionToWheelAngle(1, 1);

        swerveMove(fLAngle, fRAngle, bLAngle, bRAngle, power);
    }

    public void swerveMove(double fLAngle, double fRAngle, double bLAngle, double bRAngle, double power) {

        angleCheck(fLAngle, robot.swerveWheels.frontLeftMotor);
        angleCheck(fRAngle, robot.swerveWheels.frontRightMotor);
        angleCheck(bLAngle, robot.swerveWheels.backLeftMotor);
        angleCheck(bRAngle, robot.swerveWheels.backRightMotor);

        double servoPositionfL = robot.swerveWheels.frontLeftMotor.wheelAngleToServoPosition();
        double servoPositionfR = robot.swerveWheels.frontRightMotor.wheelAngleToServoPosition();
        double servoPositionbL = robot.swerveWheels.backLeftMotor.wheelAngleToServoPosition();
        double servoPositionbR = robot.swerveWheels.backRightMotor.wheelAngleToServoPosition();

        robot.setWheelServoPosition(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

        double frontLeftPower = robot.swerveWheels.frontLeftMotor.modifier * power;
        double frontRightPower = robot.swerveWheels.frontRightMotor.modifier * power;
        double backLeftPower = robot.swerveWheels.backLeftMotor.modifier * power;
        double backRightPower = robot.swerveWheels.backRightMotor.modifier * power;

        robot.setWheelMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void angleCheck(double goal, BuilderSkystoneHardware.SwerveWheel swerveWheel) {

        double start = swerveWheel.targetAngle;

        goal = getNormalizedAngle(goal);

        double dAngleForward = getNormalizedAngle(goal - start);
        double targetAngleForward = dAngleForward + start;
        boolean forwardPossible = (swerveWheel.minWheelAngle <= targetAngleForward && targetAngleForward <= swerveWheel.maxWheelAngle);

        double dAngleBackward = getNormalizedAngle(dAngleForward + 180);
        double targetAngleBackward = dAngleBackward + start;
        boolean backwardPossible = (swerveWheel.minWheelAngle <= targetAngleBackward && targetAngleBackward <= swerveWheel.maxWheelAngle);

        boolean goForward;

        if (forwardPossible && backwardPossible) {
            goForward = (Math.abs(dAngleForward) < Math.abs(dAngleBackward));
        } else {
            goForward = forwardPossible;
        }

        double targetAngle;
        int modifier;

        if (goForward) {
            targetAngle = targetAngleForward;
            modifier = 1;

        } else {
            targetAngle = targetAngleBackward;
            modifier = -1;
        }

        swerveWheel.setTargetAndModifier(targetAngle, modifier);
    }

    public void goToAngle(double angleStart, double angleGoal) {


        double angleCurrent = angleStart;

        while ((Math.abs(angleGoal - angleCurrent) > GO_TO_ANGLE_BUFFER) && opModeIsActive()) {
            angleCurrent = robot.getAngle();
            double power = getPower(angleCurrent, angleGoal, angleStart);
            swerveTurn(power);

            telemetry.addData("Start Angle ", "%.1f", angleStart);
            telemetry.addData("Goal Angle  ", "%.1f", angleGoal);
            telemetry.addData("Cur Angle   ", "%.1f", angleCurrent);
            telemetry.addData("Remain Angle", "%.1f", AngleUnit.normalizeDegrees(angleGoal - angleCurrent));
            telemetry.addData("Power       ", "%.2f", power);
            telemetry.update();

        }
        swerveTurn(0);
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
}
