package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;

@TeleOp(name = "SKYSTONE TELEOP", group = "competition")
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

        public void setTargetAndModifier(double targetAngle, int modifier) {
            this.targetAngle = targetAngle;
            this.modifier = modifier;
        }

        public double wheelAngleToServoPosition(double wheelAngle) {
            double servoAngle = wheelAngleToServoAngle(wheelAngle);
            double servoPosition = servoAngleToServoPosition(servoAngle);

            return servoPosition;

        }
        public double wheelAngleToServoPosition() {

            return wheelAngleToServoPosition(targetAngle);

        }

        public double wheelAngleToServoAngle(double wheelAngle) {
            double servoAngle = wheelAngle / WHEEL_SERVO_GEAR_RATIO;
            return servoAngle;
        }

        public double servoAngleToServoPosition(double servoAngle) {
            double servoPosition = (servoAngle / SERVO_MAX_ANGLE) + offset;
            return servoPosition;
        }

        public SwerveWheel(double offset) {
            this.offset = offset;
        }
    }

    public class SwerveWheels {
        SwerveWheel frontLeftMotor = new SwerveWheel(FRONT_LEFT_OFFSET);
        SwerveWheel frontRightMotor = new SwerveWheel(FRONT_RIGHT_OFFSET);
        SwerveWheel backLeftMotor = new SwerveWheel(BACK_LEFT_OFFSET);
        SwerveWheel backRightMotor = new SwerveWheel(BACK_RIGHT_OFFSET);

        public void setTargetAndModifier(double targetAngle, int modifier) {
            frontLeftMotor.setTargetAndModifier(targetAngle, modifier);
            frontRightMotor.setTargetAndModifier(targetAngle, modifier);
            backLeftMotor.setTargetAndModifier(targetAngle, modifier);
            backRightMotor.setTargetAndModifier(targetAngle, modifier);
        }
    }

    SkystoneHardware robot = new SkystoneHardware();
    SwerveWheels swerveWheels = new SwerveWheels();

    @Override
    public void init() {
        robot.init(hardwareMap);
        //The y position is -1 to correct the joystick directions
        swerveStraight(0, 1, 0);
    }

    @Override
    public void loop()  {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        double radius = Math.sqrt(Math.pow(joystickPositionX, 2) + Math.pow(joystickPositionY, 2));

        telemetry.addData("X", joystickPositionX);
        telemetry.addData("Y", joystickPositionY);

        if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
            swerveTurn(gamepad1.right_stick_x);

        } else if (radius > .2) {
            double power = getPower(joystickPositionX, joystickPositionY);
            swerveStraight(joystickPositionX, joystickPositionY, power);
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

        telemetry.addData("frontLeft", String.format("tarAngle: %.2f   mod:%d",
                swerveWheels.frontLeftMotor.targetAngle, swerveWheels.frontLeftMotor.modifier));
        telemetry.addData("frontRight", String.format("tarAngle: %.2f   mod:%d",
                swerveWheels.frontRightMotor.targetAngle, swerveWheels.frontRightMotor.modifier));
        telemetry.addData("backLeft", String.format("tarAngle: %.2f   mod:%d",
                swerveWheels.backLeftMotor.targetAngle, swerveWheels.backLeftMotor.modifier));
        telemetry.addData("backRight", String.format("tarAngle: %.2f   mod:%d",
                swerveWheels.backRightMotor.targetAngle, swerveWheels.backRightMotor.modifier));

        telemetry.update();


    }

    /*This method finds our desired angle based on the joysticks. We want out robot's wheels to
    follow the position of our joystick, so we find the angle of our joysticks position like it is
    a position on the coordinate plane
     */
    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = radiansDegreesTranslation(wheelAngleRad) - 90;
        double wheelAngleStandarized = AngleUtilities.getPositiveNormalizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }

    public double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;

    }

    public double getPower(double x, double y) {
        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        if (gamepad1.left_bumper) {
            power = 0;
        }
        return power;
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

        telemetry.addData("Power", power);
    }

    public void setPower(double joystickPositionX, double joystickPositionY, double modifier) {

        double power = modifier * getPower(joystickPositionX, joystickPositionY);

        robot.backRight.setPower(-power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(-power);

        telemetry.addData("Power", power);

    }

    public void swerveStraight(double joystickPositionX, double joystickPositionY, double power) {

        double joyWheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);

        swerveMove(joyWheelAngle, joyWheelAngle, joyWheelAngle, joyWheelAngle, power);
    }

    public void swerveTurn(double joyStickRightPosX) {

        //Math mod????????
        double hardCodeThis = Math.sqrt(2) / 2;
        double fLAngle = joystickPositionToWheelAngle(-hardCodeThis, -hardCodeThis);
        double fRAngle = joystickPositionToWheelAngle(-hardCodeThis, hardCodeThis);
        double bLAngle = joystickPositionToWheelAngle(hardCodeThis, -hardCodeThis);
        double bRAngle = joystickPositionToWheelAngle(hardCodeThis, hardCodeThis);

        swerveMove(fLAngle, fRAngle, bLAngle, bRAngle, joyStickRightPosX);
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

        telemetry.addData("servoPotionFl:", servoPositionfL);
        telemetry.addData("servoPositionfR:", servoPositionfR);
        telemetry.addData("servoPositionbL:", servoPositionbL);
        telemetry.addData("servoPositionbR:", servoPositionbR);

        setAllServos(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);
        setPower(power, 0, -Math.signum(power));
    }

    public void setAllServos(double fLPos, double fRPos, double bLPos, double bRPos) {
        robot.servoFrontRight.setPosition(fRPos);
        robot.servoBackRight.setPosition(bRPos);
        robot.servoFrontLeft.setPosition(fLPos);
        robot.servoBackLeft.setPosition(bLPos);

        telemetry.addData("Front Left Position", fLPos);
        telemetry.addData("Front Right Position", fRPos);
        telemetry.addData("Back Right Position", bRPos);
        telemetry.addData(" Back Left Position", bLPos);
    }

    public void angleCheck(double goal) {
        angleCheck(goal, this.swerveWheels.frontLeftMotor);
        angleCheck(goal, this.swerveWheels.frontRightMotor);
        angleCheck(goal, this.swerveWheels.backLeftMotor);
        angleCheck(goal, this.swerveWheels.backRightMotor);
    }

    public void angleCheck(double goal, SwerveWheel swerveWheel) {
        double start = swerveWheel.targetAngle;
        telemetry.addData("start", start);
        telemetry.addData("UnNormalized Goal", goal);

        goal = getNormalizedAngle(goal);

        telemetry.addData("Normalized Goal", goal);

        double dAngleForward = getNormalizedAngle(goal - start + 180);
        double targetAngleForward = dAngleForward + start;
        boolean forwardPossible = (targetAngleForward <= WHEEL_MAX_ANGLE && targetAngleForward >= SERVO_MIN_ANGLE);

        telemetry.addData("dAngleForward", dAngleForward);
        telemetry.addData("targetAngleForward", targetAngleForward);
        telemetry.addData("forward Possible", forwardPossible);

        double dAngleBackward = getNormalizedAngle(dAngleForward + 180);
        double targetAngleBackward = dAngleBackward + start;
        boolean backwardPossible = (targetAngleBackward <= WHEEL_MAX_ANGLE && targetAngleBackward >= SERVO_MIN_ANGLE);

        telemetry.addData("dAngleBackward", dAngleBackward);
        telemetry.addData("targetAngleBackward", targetAngleBackward);
        telemetry.addData("back Possible", backwardPossible);

        boolean goForward = true;

        double targetAngle;
        int modifier;

        if (forwardPossible && backwardPossible) {
            if (Math.abs(dAngleForward) < Math.abs(dAngleBackward)) {
                goForward = true;
            } else {
                goForward = false;
            }
        } else if (forwardPossible) {
            goForward = true;
        } else {
            goForward = false;
        }

        telemetry.addData("goForward", goForward);

        if (goForward) {
            targetAngle = targetAngleForward;
            modifier = 1;

        } else {
            targetAngle = targetAngleBackward;
            modifier = -1;
        }

        telemetry.addData("modifier", modifier);
        swerveWheel.setTargetAndModifier(targetAngle, modifier);
    }
}

