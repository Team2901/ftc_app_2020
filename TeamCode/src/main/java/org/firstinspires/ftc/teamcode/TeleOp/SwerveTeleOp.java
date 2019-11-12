package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

@TeleOp (name = "Swerve Teleop")
public class SwerveTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = 24 / 80;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 12.75;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan((.5 * WIDTH_OF_ROBOT) / (.5 * LENGTH_OF_ROBOT));
    public final static int SERVO_MAX_ANGLE = 245;
    public final static int SERVO_MIN_ANGLE = 0;
    public final static double FRONT_LEFT_OFFSET = .1;
    public final static double BACK_LEFT_OFFSET = .1;
    public final static double FRONT_RIGHT_OFFSET = .08;
    public final static double BACK_RIGHT_OFFSET = .1;
    Servo servoFrontLeft;
    public double currentAngle = 0;

    private boolean isLastRightBumperPressed;
    private boolean isLastLeftBumperPressed;
    private boolean isLastRightTriggerPressed;
    private boolean isLastLeftTriggerPressed;

    enum WheelPosition {
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
        setDriveServoPosition(0);
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        double radius = Math.sqrt(Math.pow(joystickPositionX, 2) + Math.pow(joystickPositionY, 2));

        Double wheelAngle = null;

        if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
            swerveTurn(gamepad1.right_stick_x);
        } else if (radius > .2) {
            wheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            setPower(joystickPositionX, joystickPositionY, 1);
        } else {
            setPower(0, 0, 1);
        }

        if (gamepad2.y) {
            wheelAngle = 0d;
        } else if (gamepad2.x) {
            wheelAngle = 90d;
        } else if (gamepad2.a) {
            wheelAngle = 180d;
        } else if (gamepad2.b) {
            wheelAngle = 270d;
        } else if (gamepad2.right_bumper && !isLastRightBumperPressed){
            wheelAngle = currentAngle + 90;
        }
        if (gamepad2.left_bumper && !isLastLeftBumperPressed) {
            wheelAngle = currentAngle - 90;
        }
        if (gamepad2.right_trigger > 0.01 && !isLastRightTriggerPressed){
            wheelAngle = currentAngle + 5;
        }
        if (gamepad2.left_trigger > 0.01 && !isLastLeftTriggerPressed) {
            wheelAngle = currentAngle - 5;
        }

        if (wheelAngle != null) {
            setDriveServoPosition(wheelAngle);
        }

        isLastRightBumperPressed = gamepad2.right_bumper;
        isLastLeftBumperPressed = gamepad2.left_bumper;
        isLastRightTriggerPressed = gamepad2.right_trigger > 0.01;
        isLastLeftTriggerPressed = gamepad2.left_trigger > 0.01;

        telemetry.update();
    }

    public void setDriveServoPosition(double wheelAngle) {

        double servoAngle = wheelAngleToServoAngle(wheelAngle);

        this.currentAngle = wheelAngle;

        telemetry.addData("wheelAngle:", wheelAngle);
        telemetry.addData("servoAngle:", servoAngle);

        double servoPositionfL = servoAngleToServoPosition(servoAngle, WheelPosition.FRONT_LEFT);
        double servoPositionfR = servoAngleToServoPosition(servoAngle, WheelPosition.FRONT_RIGHT);
        double servoPositionbL = servoAngleToServoPosition(servoAngle, WheelPosition.BACK_LEFT);
        double servoPositionbR = servoAngleToServoPosition(servoAngle, WheelPosition.FRONT_RIGHT);

        telemetry.addData("servoPotionFl:", servoPositionfL);
        telemetry.addData("servoPositionfR:", servoPositionfR);
        telemetry.addData("servoPositionbL:", servoPositionbL);
        telemetry.addData("servoPositionbR:", servoPositionbR );

        setAllServos(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);
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

    /* This is a helper function that is used in 2 other methods.  This class takes the goal angle
    of "wheel angle" and shifts it to what the desired angle of the servo. This is the mathematical
    reason for the gear ratio
    */
    public double wheelAngleToServoAngle(double wheelAngle) {
        double servoAngle = WHEEL_SERVO_GEAR_RATIO * wheelAngle;
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

        robot.backRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(power);

        telemetry.addData("Power", power);

    }

    public void swerveTurn(double joyStickRightPosX) {

        double fRPos = 90 + TURN_ANGLE;
        double fLPos = 90 - TURN_ANGLE;
        double bLPos = 270 + TURN_ANGLE;
        double bRPos = 270 - TURN_ANGLE;

        double fRAngle = wheelAngleToServoAngle(fRPos);
        double fLAngle = wheelAngleToServoAngle(fLPos);
        double blAngle = wheelAngleToServoAngle(bLPos);
        double bRAngle = wheelAngleToServoAngle(bRPos);

        setAllServos(servoAngleToServoPosition(fRAngle, WheelPosition.FRONT_RIGHT),
                servoAngleToServoPosition(fLAngle, WheelPosition.FRONT_LEFT),
                servoAngleToServoPosition(blAngle, WheelPosition.BACK_LEFT),
                servoAngleToServoPosition(bRAngle, WheelPosition.BACK_RIGHT));

        setPower(joyStickRightPosX, 0, Math.signum(joyStickRightPosX));

    }

    public void setAllServos(double fLPos, double fRPos, double bLPos, double bRPos) {
        robot.servoFrontRight.setPosition(fRPos);
        robot.servoBackRight.setPosition(bRPos);
        robot.servoFrontLeft.setPosition(fLPos);
        robot.servoBackLeft.setPosition(bLPos);
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