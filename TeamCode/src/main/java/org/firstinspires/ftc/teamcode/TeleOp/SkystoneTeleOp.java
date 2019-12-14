package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;

@SuppressLint("DefaultLocale")
@TeleOp(name = "SKYSTONE TELEOP 2", group = "competition")
public class SkystoneTeleOp extends OpMode {

    public final static double WHEEL_SERVO_GEAR_RATIO = .3;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan(WIDTH_OF_ROBOT/ LENGTH_OF_ROBOT);
    public final static int SERVO_MAX_ANGLE = 2727;
    public final static int SERVO_MIN_ANGLE = 0;
    public final static double FRONT_LEFT_OFFSET = .11;
    public final static double FRONT_RIGHT_OFFSET = .13;
    public final static double BACK_LEFT_OFFSET = .1;
    public final static double BACK_RIGHT_OFFSET = .11;

    public final static double WHEEL_MAX_ANGLE = SERVO_MAX_ANGLE * WHEEL_SERVO_GEAR_RATIO;

    public class SwerveWheel {
        double targetAngle = 0;
        int modifier = 1;
        double offset = 0;

        public SwerveWheel(double offset) {
            this.offset = offset;
        }

        public void setTargetAndModifier(double targetAngle, int modifier) {
            this.targetAngle = targetAngle;
            this.modifier = modifier;
        }

        public double wheelAngleToServoPosition(double wheelAngle) {
            double servoAngle = wheelAngleToServoAngle(wheelAngle);
            return servoAngleToServoPosition(servoAngle);
        }

        public double wheelAngleToServoPosition() {
            return wheelAngleToServoPosition(targetAngle);
        }

        public double wheelAngleToServoAngle(double wheelAngle) {
            return wheelAngle / WHEEL_SERVO_GEAR_RATIO;
        }

        public double servoAngleToServoPosition(double servoAngle) {
            return (servoAngle / SERVO_MAX_ANGLE) + offset;
        }
    }

    public class SwerveWheels {
        SwerveWheel frontLeftMotor = new SwerveWheel(FRONT_LEFT_OFFSET);
        SwerveWheel frontRightMotor = new SwerveWheel(FRONT_RIGHT_OFFSET);
        SwerveWheel backLeftMotor = new SwerveWheel(BACK_LEFT_OFFSET);
        SwerveWheel backRightMotor = new SwerveWheel(BACK_RIGHT_OFFSET);
    }

    public SkystoneHardware robot = new SkystoneHardware();
    public SwerveWheels swerveWheels = new SwerveWheels();

    @Override
    public void init() {
        robot.init(hardwareMap);
        swerveStraight(0, 0);
    }

    @Override
    public void loop()  {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.right_stick_x) > .1) {
            double power = getPower(gamepad1.right_stick_x, 0, gamepad1.left_bumper);
            swerveTurn(power);
        } else if (AngleUtilities.getRadius(joystickPositionX, joystickPositionY) > .2) {
            double power = getPower(joystickPositionX, joystickPositionY, gamepad1.left_bumper);
            double joyWheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            swerveStraight(joyWheelAngle, power);
            setPower(power);
        } else {
            setPower(0);
        }

        if (gamepad2.left_trigger > .2) {
            robot.lift.setPower(-.5);
        } else if (gamepad2.right_trigger > .2) {
            robot.lift.setPower(1);
        } else {
            robot.lift.setPower(0);
        }

        if (gamepad2.right_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() + .005);
        } else if (gamepad2.left_bumper) {
            robot.crane.setPosition(robot.crane.getPosition() - .005);
        }

        if (gamepad2.x) {
            robot.wrist.setPosition(robot.wrist.getPosition() + .01);
        } else if (gamepad2.y) {
            robot.wrist.setPosition(robot.wrist.getPosition() - .01);
        }

        if (gamepad2.a) {
            robot.jaw.setPosition(robot.jaw.getPosition() + .01);
        } else if (gamepad2.b) {
            robot.jaw.setPosition(robot.jaw.getPosition() - .01);
        }

        telemetry.addData("FL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                this.swerveWheels.frontLeftMotor.targetAngle, this.swerveWheels.frontLeftMotor.modifier, this.swerveWheels.frontLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("FR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                this.swerveWheels.frontRightMotor.targetAngle, this.swerveWheels.frontRightMotor.modifier, this.swerveWheels.frontRightMotor.wheelAngleToServoPosition()));
        telemetry.addData("BL", String.format("angle: %.2f, mod: %d, pos: %.2f",
                this.swerveWheels.backLeftMotor.targetAngle, this.swerveWheels.backLeftMotor.modifier, this.swerveWheels.backLeftMotor.wheelAngleToServoPosition()));
        telemetry.addData("BR", String.format("angle: %.2f, mod: %d, pos: %.2f",
                this.swerveWheels.backRightMotor.targetAngle, this.swerveWheels.backRightMotor.modifier, this.swerveWheels.backRightMotor.wheelAngleToServoPosition()));

        telemetry.update();
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

        angleCheck(fLAngle, this.swerveWheels.frontLeftMotor);
        angleCheck(fRAngle, this.swerveWheels.frontRightMotor);
        angleCheck(bLAngle, this.swerveWheels.backLeftMotor);
        angleCheck(bRAngle, this.swerveWheels.backRightMotor);

        double servoPositionfL = this.swerveWheels.frontLeftMotor.wheelAngleToServoPosition();
        double servoPositionfR = this.swerveWheels.frontRightMotor.wheelAngleToServoPosition();
        double servoPositionbL = this.swerveWheels.backLeftMotor.wheelAngleToServoPosition();
        double servoPositionbR = this.swerveWheels.backRightMotor.wheelAngleToServoPosition();

        setAllServos(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

        setPower(power);
    }

    public void setAllServos(double fLPos, double fRPos, double bLPos, double bRPos) {
        robot.servoFrontRight.setPosition(fRPos);
        robot.servoBackRight.setPosition(bRPos);
        robot.servoFrontLeft.setPosition(fLPos);
        robot.servoBackLeft.setPosition(bLPos);
    }

    public void setPower(double power) {

        double frontLeftPower = swerveWheels.frontLeftMotor.modifier * power;
        double frontRightPower = swerveWheels.frontRightMotor.modifier * power;
        double backLeftPower = swerveWheels.backLeftMotor.modifier * power;
        double backRightPower = swerveWheels.backRightMotor.modifier * power;

        robot.backRight.setPower(-backRightPower);
        robot.backLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.frontLeft.setPower(-frontLeftPower);
    }

    public void angleCheck(double goal, SwerveWheel swerveWheel) {

        double start = swerveWheel.targetAngle;

        goal = getNormalizedAngle(goal);

        double dAngleForward = getNormalizedAngle(goal - start);
        double targetAngleForward = dAngleForward + start;
        boolean forwardPossible = (targetAngleForward <= WHEEL_MAX_ANGLE && targetAngleForward >= SERVO_MIN_ANGLE);

        double dAngleBackward = getNormalizedAngle(dAngleForward + 180);
        double targetAngleBackward = dAngleBackward + start;
        boolean backwardPossible = (targetAngleBackward <= WHEEL_MAX_ANGLE && targetAngleBackward >= SERVO_MIN_ANGLE);

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
}

