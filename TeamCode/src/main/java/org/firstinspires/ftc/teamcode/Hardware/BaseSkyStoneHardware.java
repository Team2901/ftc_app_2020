package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;
import org.firstinspires.ftc.teamcode.Utility.FileUtilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;


public class
BaseSkyStoneHardware {

    String CONFIG_FILENAME = "servo_offset_config.txt";
    List<Double> offsets = new ArrayList<>();

    public static final double GRABBER_MIN = 0.25;
    public static final double GRABBER_MAX = 0.75;
    public static final double ROBOT_FRONT_ANGLE = 0;
    public static final double ROBOT_RIGHT_ANGLE = -90;
    public static final double ROBOT_LEFT_ANGLE = 90;
    public static final double OPEN_JAW = 1;
    public static final double CLOSED_JAW = 0;
    public static final double LIFT_STEP = 750;

    public static final String WEB_CAM_NAME = "Webcam 1";

    public final double inchesToEncoder;
    public double wheelServoGearRatio;
    public double widthOfRobot;
    public double lengthOfRobot;
    public double turnAngle;
    public double servoMaxAngle;
    public double frontLeftOffset;
    public double frontRightOffset;
    public double backLeftOffset;
    public double backRightOffset;
    public Servo leftGrabber;
    public Servo rightGrabber;

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_BUTTER = "Stone";
    public static final String LABEL_SKY_BUTTER = "Skystone";

    public BaseSkyStoneHardware(double widthOfRobot,
                                double lengthOfRobot,
                                double wheelServoGearRatio,
                                double servoMaxAngle,
                                double frontLeftOffset,
                                double frontRightOffset,
                                double backLeftOffset,
                                double backRightOffset,
                                double inchesToEncoder
    ) {
        this.inchesToEncoder = inchesToEncoder;
        this.wheelServoGearRatio = wheelServoGearRatio;
        this.widthOfRobot = widthOfRobot;
        this.lengthOfRobot = lengthOfRobot;
        this.servoMaxAngle = servoMaxAngle;
        this.frontLeftOffset = frontLeftOffset;
        this.frontRightOffset = frontRightOffset;
        this.backLeftOffset = backLeftOffset;
        this.backRightOffset = backRightOffset;
        this.turnAngle = Math.atan(widthOfRobot/lengthOfRobot);

    }

    public class SwerveWheel {
        public double targetAngle = 0;
        public int modifier = 1;
        public double offset = 0;
        public double minWheelAngle=0;
        public double maxWheelAngle = 0;

        public SwerveWheel(double offset) {
            this.offset = offset;
            minWheelAngle = servoPositionToWheelAngle(0);
            maxWheelAngle = servoPositionToWheelAngle(1);
        }

        public void setTargetAndModifier(double targetAngle, int modifier) {
            this.targetAngle = targetAngle;
            this.modifier = modifier;
        }

        public double wheelAngleToServoPosition(double wheelAngle) {
            /*
            y=mx+b
            ServoPosition = [gearRatio*wheelAngle]/ServoMaxAngle] + offset
            wheelAngle is x
            */
            double servoAngle = wheelAngleToServoAngle(wheelAngle);
            return servoAngleToServoPosition(servoAngle);
        }

        public double wheelAngleToServoPosition() {
            return wheelAngleToServoPosition(targetAngle);
        }

        public double wheelAngleToServoAngle(double wheelAngle) {
            return wheelAngle / wheelServoGearRatio;
        }

        public double servoAngleToServoPosition(double servoAngle) {
            return (servoAngle / servoMaxAngle) + offset;
        }

        public double servoPositionToWheelAngle(double servoPosition){
            return (wheelServoGearRatio*servoMaxAngle)*(servoPosition-offset);

        }
    }

    public class SwerveWheels {
        public SwerveWheel frontLeftMotor = new SwerveWheel(frontLeftOffset);
        public SwerveWheel frontRightMotor = new SwerveWheel(frontRightOffset);
        public SwerveWheel backLeftMotor = new SwerveWheel(backLeftOffset);
        public SwerveWheel backRightMotor = new SwerveWheel(backRightOffset);
    }

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

    public Servo crane;
    public Servo jaw;
    public Servo wrist;

    public SwerveWheels swerveWheels;

    //Sensors and Things
    public BNO055IMU imu;
    public IntegratingGyroscope gyroscope;

    public double offset = 0;

    public TensorFlowCamera webCamera = new TensorFlowCamera();

    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //Inititialize all Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftGrabber = hwMap.get(Servo.class, "leftGrabber");
        rightGrabber = hwMap.get(Servo.class, "rightGrabber");
        leftGrabber.setDirection(Servo.Direction.REVERSE);

        /*Servos Aren't lined up so these aren't the same*/
        //leftGrabber.setPosition(GRABBER_MIN);
        //rightGrabber.setPosition(GRABBER_MIN);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize all servos
        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
        servoFrontRight = hardwareMap.servo.get("servoFrontRight");
        servoBackLeft = hardwareMap.servo.get("servoBackLeft");
        servoBackRight = hardwareMap.servo.get("servoBackRight");

        crane = hardwareMap.servo.get("crane");
        jaw = hardwareMap.servo.get("jaw");
        wrist = hardwareMap.servo.get("wrist");

        // crane is skipping, dont move it on init
        //crane.setPosition(.05);

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

        try {
            offsets = FileUtilities.readDoubleConfigFile(CONFIG_FILENAME);
        } catch (IOException e) {
            offsets = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                offsets.add(0.0);
            }
        }

        if (offsets.size() < 4) {
            offsets = new ArrayList<>();
            for (int i = 0; i < 4; i++) {
                offsets.add(0.0);
            }
        }

        frontLeftOffset = offsets.size() > 0 ? offsets.get(0) : 0.0;
        frontRightOffset = offsets.size() > 1 ? offsets.get(1) : 0.0;
        backLeftOffset = offsets.size() > 2 ? offsets.get(2) : 0.0;
        backRightOffset = offsets.size() > 3 ? offsets.get(3) : 0.0;

        this.swerveWheels = new SwerveWheels();
    }

    public String initWebCamera(HardwareMap hardwareMap) {
        return webCamera.initWebCamera(hardwareMap, WEB_CAM_NAME,.8, TFOD_MODEL_ASSET, LABEL_BUTTER, LABEL_SKY_BUTTER);
    }

    public double getRawAngle() {
        Orientation orientation = gyroscope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUtilities.getNormalizedAngle(orientation.firstAngle);
    }

    public double getAngle() {
        return AngleUtilities.getNormalizedAngle(getRawAngle() + offset);
    }

    public void setWheelMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
    }

    public void setWheelServoPosition(double fLPos, double fRPos, double bLPos, double bRPos) {
        servoFrontRight.setPosition(fRPos);
        servoBackRight.setPosition(bRPos);
        servoFrontLeft.setPosition(fLPos);
        servoBackLeft.setPosition(bLPos);
    }

    public void setWheelMotorMode(DcMotor.RunMode runMode) {
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
    }

    public void setGrabberPositition (double leftGrabberPos, double rightGrabberPos){
        leftGrabber.setPosition(leftGrabberPos);
        rightGrabber.setPosition(rightGrabberPos);
    }

    public boolean wheelsAreBusy() {
        return frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy();

    }

    public void wait(int milliseconds, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.milliseconds() < milliseconds) {

        }
    }

    public boolean isOkayToOpen() {
        if(crane.getPosition()> .3) {
            if (lift.getCurrentPosition() == 0) {
                if (wrist.getPosition() > .25) {
                    return false;
                }
                return true;
            } else {
                return true;
            }
        }
        return true;
    }

    public void moveStraight (double setPower, int targetPosition){

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition(targetPosition);
        frontLeft.setTargetPosition(targetPosition);
        backLeft.setTargetPosition(targetPosition);
        backRight.setTargetPosition(targetPosition);

        setWheelMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setWheelMotorPower(setPower, setPower, setPower, setPower);
        //call whileMotorisBusy



    }

    public void parkingLowRisk(){
        //Step one: turn wheels 90 degrees counterclockwise and go forward 1 ft.
        moveStraight(0.5,1);
        //Step two: turn wheels 90 degrees clockwise and go forward 1 ft.
        moveStraight(0.5,1);
        //Step three: turn wheels 90 degrees counterclockwise and go forward 1.5 ft.
        moveStraight(0.5,2);
        //Step four: stop
    }

    public void setWheelTargetPositions(int position){
        frontLeft.setTargetPosition(position*swerveWheels.frontLeftMotor.modifier);
        frontRight.setTargetPosition(position*swerveWheels.frontRightMotor.modifier);
        backLeft.setTargetPosition(position*swerveWheels.backLeftMotor.modifier);
        backRight.setTargetPosition(position*swerveWheels.backRightMotor.modifier);
    }

    public void angleCheck(double goal, BaseSkyStoneHardware.SwerveWheel swerveWheel) {

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

    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = AngleUtilities.radiansDegreesTranslation(wheelAngleRad) - 90;
        return AngleUtilities.getPositiveNormalizedAngle(wheelAngle);

    }

    public void swerveStraight(double joyWheelAngle, double power) {
        swerveMove(joyWheelAngle, joyWheelAngle, joyWheelAngle, joyWheelAngle, power);
    }

    public void swerveTurn(double power) {

        double fLAngle = joystickPositionToWheelAngle(1, 1);
        double fRAngle = joystickPositionToWheelAngle(1, -1);
        double bLAngle = joystickPositionToWheelAngle(-1, 1);
        double bRAngle = joystickPositionToWheelAngle(-1, -1);

        swerveMove(fLAngle, fRAngle, bLAngle, bRAngle, power);
    }

    public void swerveMove(double fLAngle, double fRAngle, double bLAngle, double bRAngle, double power) {

        angleCheck(fLAngle, swerveWheels.frontLeftMotor);
        angleCheck(fRAngle, swerveWheels.frontRightMotor);
        angleCheck(bLAngle, swerveWheels.backLeftMotor);
        angleCheck(bRAngle, swerveWheels.backRightMotor);

        double servoPositionfL = swerveWheels.frontLeftMotor.wheelAngleToServoPosition();
        double servoPositionfR = swerveWheels.frontRightMotor.wheelAngleToServoPosition();
        double servoPositionbL = swerveWheels.backLeftMotor.wheelAngleToServoPosition();
        double servoPositionbR = swerveWheels.backRightMotor.wheelAngleToServoPosition();

        setWheelServoPosition(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

        double frontLeftPower = swerveWheels.frontLeftMotor.modifier * power;
        double frontRightPower = swerveWheels.frontRightMotor.modifier * power;
        double backLeftPower = swerveWheels.backLeftMotor.modifier * power;
        double backRightPower = swerveWheels.backRightMotor.modifier * power;

        setWheelMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}

