package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

@TeleOp (name = "SKYSTONE TELEOP" ,group = "competition")
public class SkystoneTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = .3;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan((.5 * WIDTH_OF_ROBOT) / (.5 * LENGTH_OF_ROBOT));
    public final static int SERVO_MAX_ANGLE = 2727;
    public final static int SERVO_MIN_ANGLE = 0;
    public final static double FRONT_LEFT_OFFSET = .11;
    public final static double BACK_LEFT_OFFSET = .1;
    public final static double FRONT_RIGHT_OFFSET = 0;
    public final static double BACK_RIGHT_OFFSET = .03;
    Servo servoFrontLeft;
    public double currentAngle = 0;


    public enum WheelPosition {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);

        int value;

        WheelPosition(int value) {
            this.value = value;
        }
    }

    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        //The y position is -1 to correct the joystick directions
        setDriveServoPosition(0, -1);
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        double radius = Math.sqrt(Math.pow(joystickPositionX, 2) + Math.pow(joystickPositionY, 2));

        telemetry.addData("X", joystickPositionX);
        telemetry.addData("Y", joystickPositionY);


        if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
            swerveTurn(gamepad1.right_stick_x);

        } else if (radius > .2) {
            currentAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            setDriveServoPosition(joystickPositionX, joystickPositionY);
            setPower(joystickPositionX, joystickPositionY, 1);
        } else {
            setPower(0, 0, 1);
        }

        if(gamepad2.left_trigger > .2){
            robot.lift.setPower(-.5);
        }else if (gamepad2.right_trigger>.2){
            robot.lift.setPower(1);
        }else{
            robot.lift.setPower(0);
        }

        if(gamepad2.right_bumper){
            robot.crane.setPosition(robot.crane.getPosition() + .01);
        }else if (gamepad2.left_bumper){
            robot.crane.setPosition(robot.crane.getPosition() - .01);
        }

        if(gamepad2.x){
            robot.wrist.setPosition(robot.wrist.getPosition() + .01);
        }else if (gamepad2.y){
            robot.wrist.setPosition(robot.wrist.getPosition() - .01);
        }

        if(gamepad2.a){
            robot.jaw.setPosition(robot.jaw.getPosition() + .01);
        }else if (gamepad2.b){
            robot.jaw.setPosition(robot.jaw.getPosition() - .01);
        }


        telemetry.update();


    }

    /* This is a helper function that is used in 2 other methods.  This class takes the goal angle
    of "wheel angle" and shifts it to what the desired angle of the servo. This is the mathematical
    reason for the gear ratio
    */
    public double wheelAngleToServoAngle(double wheelAngle) {
        double servoAngle = wheelAngle / WHEEL_SERVO_GEAR_RATIO;
        telemetry.addData("Goal Angle", wheelAngle);
        return servoAngle;
    }

    public double servoAngleToServoPosition(double servoAngle, WheelPosition wheelPosition) {
        switch (wheelPosition) {
            case BACK_LEFT:
                return servoAngleToServoPositionBL(servoAngle);

            case BACK_RIGHT:
                return servoAngleToServoPositionBR(servoAngle);

            case FRONT_LEFT:
                return servoAngleToServoPositionFL(servoAngle);

            case FRONT_RIGHT:
            default:
                return servoAngleToServoPositionFR(servoAngle);
        }
    }

    /*This is the next step in the process and takes the desired servo angle and divide it by the
    total servo angles so we can get a position between 0 and 1 for our desired location
     */
    public double servoAngleToServoPositionFL(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) + FRONT_LEFT_OFFSET;
        return servoPosition;
    }

    public double servoAngleToServoPositionFR(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) + FRONT_RIGHT_OFFSET;
        return servoPosition;
    }

    public double servoAngleToServoPositionBL(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) + BACK_LEFT_OFFSET;
        return servoPosition;
    }

    public double servoAngleToServoPositionBR(double servoAngle) {
        double servoPosition = (servoAngle / SERVO_MAX_ANGLE) + BACK_RIGHT_OFFSET;

        return servoPosition;
    }

    /*This method finds our desired angle based on the joysticks. We want out robot's wheels to
    follow the position of our joystick, so we find the angle of our joysticks position like it is
    a position on the coordinate plane
     */
    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = radiansDegreesTranslation(wheelAngleRad) - 90;
        double wheelAngleStandarized = standardizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }

    public double joystickPositionToWheelAngle(double currentGoal) {
        double wheelAngleRad = currentGoal;
        double wheelAngle = radiansDegreesTranslation(wheelAngleRad) - 90;
        double wheelAngleStandarized = standardizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }

    public double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;

    }

    //Converting -180 to 180 to 0 to 360
    public double standardizedAngle(double angle) {
        return (angle + 360) % 360;
    }

    public double getPower(double x, double y) {
        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        if (gamepad1.left_bumper) {
            power = 0;
        }
        return power;
    }

    public void setPower(double joystickPositionX, double joystickPositionY, double modifier) {

        double power = modifier * getPower(joystickPositionX, joystickPositionY);

        robot.backRight.setPower(-power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(-power);

        telemetry.addData("Power", power);

    }

    public void setDriveServoPosition(double joystickPositionX, double joystickPositionY) {
//happy with this. It's doing what its supposed to
        double wheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);

        this.currentAngle = wheelAngle;

        telemetry.addData("current angle:", currentAngle);

        /*
        double servoPositionfL = wheelAngleToServoPosition(wheelAngle,
                WheelPosition.FRONT_LEFT);
        double servoPositionfR = wheelAngleToServoPosition(wheelAngle,
                WheelPosition.FRONT_RIGHT);
        double servoPositionbL = wheelAngleToServoPosition(wheelAngle,
                WheelPosition.BACK_LEFT);
        double servoPositionbR = wheelAngleToServoPosition(wheelAngle,
                WheelPosition.FRONT_RIGHT);
        */

        double servoPositionfL = joystickToServoPosition(wheelAngle, WheelPosition.FRONT_LEFT);
        double servoPositionfR = joystickToServoPosition(wheelAngle, WheelPosition.FRONT_RIGHT);
        double servoPositionbL = joystickToServoPosition(wheelAngle, WheelPosition.BACK_LEFT);
        double servoPositionbR = joystickToServoPosition(wheelAngle, WheelPosition.BACK_RIGHT);

        telemetry.addData("servoPotionFl:", servoPositionfL);
        telemetry.addData("servoPositionfR:", servoPositionfR);
        telemetry.addData("servoPositionbL:", servoPositionbL);
        telemetry.addData("servoPositionbR:", servoPositionbR);


        setAllServos(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

    }

    public void setDriveServoPosition(double currentGoal) {
        double servoPositionfL = joystickToServoPosition(currentGoal,
                WheelPosition.FRONT_LEFT);
        double servoPositionfR = joystickToServoPosition(currentGoal,
                WheelPosition.FRONT_RIGHT);
        double servoPositionbL = joystickToServoPosition(currentGoal,
                WheelPosition.BACK_LEFT);
        double servoPositionbR = joystickToServoPosition(currentGoal,
                WheelPosition.FRONT_RIGHT);


        setAllServos(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);
    }

    public void swerveTurn(double joyStickRightPosX) {

        //Math mod????????
        double hardCodeThis = Math.sqrt(2) / 2;
        double fLPos = joystickPositionToWheelAngle(-hardCodeThis, -hardCodeThis);
        double fRPos = joystickPositionToWheelAngle(-hardCodeThis, hardCodeThis);
        double bLPos = joystickPositionToWheelAngle(hardCodeThis, -hardCodeThis);
        double bRPos = joystickPositionToWheelAngle(hardCodeThis, hardCodeThis);

        double servoPositionfL = joystickToServoPosition(fLPos, WheelPosition.FRONT_LEFT);
        double servoPositionfR = joystickToServoPosition(fRPos, WheelPosition.FRONT_RIGHT);
        double servoPositionbL = joystickToServoPosition(bLPos, WheelPosition.BACK_LEFT);
        double servoPositionbR = joystickToServoPosition(bRPos, WheelPosition.BACK_RIGHT);

        telemetry.addData("servoPotionFl:", servoPositionfL);
        telemetry.addData("servoPositionfR:", servoPositionfR);
        telemetry.addData("servoPositionbL:", servoPositionbL);
        telemetry.addData("servoPositionbR:", servoPositionbR);


        setAllServos(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

        setPower(joyStickRightPosX,0, -Math.signum(joyStickRightPosX));

    }

    public double joystickToServoPosition(double wheelAngle, WheelPosition wheelPosition) {
        // double wheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
        double servoAngle = wheelAngleToServoAngle(wheelAngle);
        double servoPosition = servoAngleToServoPosition(servoAngle, wheelPosition);

        return servoPosition;

    }
//bad one
   /* public double joystickToServoPosition(double currentGoal, WheelPosition wheelPosition) {
        double wheelAngle = joystickPositionToWheelAngle(currentGoal);
        double servoAngle = wheelAngleToServoAngle(wheelAngle);
        double servoPosition = servoAngleToServoPosition(servoAngle, wheelPosition);
        return servoPosition;
    }*/

    public double wheelAngleToServoPosition(double angle, WheelPosition wheelPosition) {
        double servoAngle = wheelAngleToServoAngle(angle);
        double servoPosition = servoAngleToServoPosition(servoAngle, wheelPosition);
        return servoPosition;
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

    public double normalizeAngle(double angle) {
        return ((angle + 180) % 360) - 180;
    }

    public double angleCheck(double start, double goal) {
        goal = normalizeAngle(goal);

        double dAngleForward = ((goal - start + 180) % 360) - 180;
        double targetAngleForward = dAngleForward + start;
        boolean forwardPossible = (targetAngleForward < SERVO_MAX_ANGLE && targetAngleForward > SERVO_MIN_ANGLE);

        double dAngleBackward = ((goal - start) % 360) - 180;
        double targetAngleBackward = dAngleBackward + start;
        boolean backwardPossible = (targetAngleBackward < SERVO_MAX_ANGLE && targetAngleBackward > SERVO_MIN_ANGLE);

        boolean goForward = true;

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

        if (goForward) {
            return targetAngleForward;
        } else {
            return targetAngleBackward;
        }
    }
}