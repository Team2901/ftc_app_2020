package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SkystoneHardware {

    final static double TICKS_PER_INCH = 2240 / (3 * Math.PI);

    public HardwareMap hardwareMap;

    //Made for a 4 wheel swerve drive system
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor lift;


    //Steering servo for their respective motor
    public Servo servoFrontLeft;
    public Servo servoFrontRight;
    public Servo servoBackLeft;
    public Servo servoBackRight;

    public Servo bridgeTickler;
    public Servo crane;
    public Servo jaw;
    public Servo wrist;

    //Sensors and Things
    public BNO055IMU imu;
    public IntegratingGyroscope gyroscope;

    //Constants and things
    public final static double WHEEL_SERVO_GEAR_RATIO = 24 / 80;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 12.75;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan((.5 * WIDTH_OF_ROBOT) / (.5 * LENGTH_OF_ROBOT));
    public final static int SERVO_MAX_ANGLE = 245;
    public final static int SERVO_MIN_ANGLE = 0;
    public final static double FRONT_LEFT_OFFSET = .1;
    public final static double BACK_LEFT_OFFSET = .1;
    public final static double FRONT_RIGHT_OFFSET = .1;
    public final static double BACK_RIGHT_OFFSET = .1;
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

    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //Inititialize all Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        lift = hardwareMap.dcMotor.get("lift");

        //Initialize all servos
        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
        servoFrontRight = hardwareMap.servo.get("servoFrontRight");
        servoBackLeft = hardwareMap.servo.get("servoBackLeft");
        servoBackRight = hardwareMap.servo.get("servoBackRight");

        bridgeTickler = hardwareMap.servo.get("bridgeTickler");
        crane = hardwareMap.servo.get("crane");
        jaw = hardwareMap.servo.get("jaw");
        wrist = hardwareMap.servo.get("wrist");

        crane.setPosition(0);

        //setAllSteeringServos(0);

        // setting up the gyroscope
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyroscope = (IntegratingGyroscope) imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void setAllSteeringServos(double position) {
        servoBackRight.setPosition(position);
        servoBackLeft.setPosition(position);
        servoFrontLeft.setPosition(position);
        servoFrontRight.setPosition(position);
    }

    public void setDriveServoPosition(double wheelAngle) {

        wheelAngle = standardizedAngle(wheelAngle-90);

        double servoAngle = wheelAngleToServoAngle(wheelAngle);

        this.currentAngle = wheelAngle;

        double servoPositionfL = servoAngleToServoPosition(servoAngle, SkystoneHardware.WheelPosition.FRONT_LEFT);
        double servoPositionfR = servoAngleToServoPosition(servoAngle, SkystoneHardware.WheelPosition.FRONT_RIGHT);
        double servoPositionbL = servoAngleToServoPosition(servoAngle, SkystoneHardware.WheelPosition.BACK_LEFT);
        double servoPositionbR = servoAngleToServoPosition(servoAngle, SkystoneHardware.WheelPosition.FRONT_RIGHT);


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
        double servoAngle = wheelAngle / WHEEL_SERVO_GEAR_RATIO;
        return servoAngle;
    }

    public double servoAngleToServoPosition(double servoAngle, SkystoneHardware.WheelPosition wheelPosition) {
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

    public double getPower(double x, double y, boolean leftBumper) {

        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        if (leftBumper) {
            power = 0;
        }
        return power;
    }

    public void setPower(double joystickPositionX, double joystickPositionY, double modifier, boolean leftBumper) {


        double power = modifier * getPower(joystickPositionX, joystickPositionY, leftBumper);
        setPower(power);
    }

    public void setPower(double power) {


        backRight.setPower(-power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(-power);


    }

    public void swerveTurn(double joyStickRightPosX, boolean leftBumper) {

        //Math mod????????

        double fRPos = TURN_ANGLE;
        double fLPos = 90 + TURN_ANGLE;
        double bLPos = 180 + TURN_ANGLE;
        double bRPos = 270 + TURN_ANGLE;

        double fRAngle = wheelAngleToServoAngle(fRPos);
        double fLAngle = wheelAngleToServoAngle(fLPos);
        double blAngle = wheelAngleToServoAngle(bLPos);
        double bRAngle = wheelAngleToServoAngle(bRPos);

        setAllServos(servoAngleToServoPosition(fLAngle, SkystoneHardware.WheelPosition.FRONT_LEFT),
                servoAngleToServoPosition(fRAngle, SkystoneHardware.WheelPosition.FRONT_RIGHT),
                servoAngleToServoPosition(blAngle, SkystoneHardware.WheelPosition.BACK_LEFT),
                servoAngleToServoPosition(bRAngle, SkystoneHardware.WheelPosition.BACK_RIGHT));

        setPower(joyStickRightPosX, 0, -Math.signum(joyStickRightPosX), leftBumper);

    }

    public void setAllServos(double fLPos, double fRPos, double bLPos, double bRPos) {
        servoFrontRight.setPosition(fRPos);
        servoBackRight.setPosition(bRPos);
        servoFrontLeft.setPosition(fLPos);
        servoBackLeft.setPosition(bLPos);
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

    public void setMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
    }

    public boolean wheelsAreBusy() {
        return frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy();

    }

    public void goToPosition(double distanceInch, LinearOpMode opMode) {
        int distanceTicks = (int) (TICKS_PER_INCH * distanceInch);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // stop and reset encoder mode

        frontRight.setTargetPosition(distanceTicks);
        frontLeft.setTargetPosition(distanceTicks);
        backRight.setTargetPosition(distanceTicks);
        backLeft.setTargetPosition(distanceTicks);
        // set motors' targetPosition

        setPower(.7);
        // set motors' power
        while (opMode.opModeIsActive() && wheelsAreBusy()) {

        }

        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wait while motors are busy

    }

    public void wait(int milliseconds, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.milliseconds() < milliseconds) {

        }
    }
}

