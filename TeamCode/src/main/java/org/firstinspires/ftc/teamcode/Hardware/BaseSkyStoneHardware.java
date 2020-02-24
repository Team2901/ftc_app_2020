package org.firstinspires.ftc.teamcode.Hardware;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@SuppressLint("DefaultLocale")
public class BaseSkyStoneHardware {

    String CONFIG_FILENAME = "servo_offset_config.txt";
    public List<Double> offsets = new ArrayList<>();

    // TODO
    final double basePowerRatio = 0.025;
    final double stallPowerRatio = 0; // 0.05;

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
    public Servo leftGrabber;
    public Servo rightGrabber;

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_BUTTER = "Stone";
    public static final String LABEL_SKY_BUTTER = "Skystone";

    public BaseSkyStoneHardware(double widthOfRobot,
                                double lengthOfRobot,
                                double wheelServoGearRatio,
                                double servoMaxAngle,
                                double inchesToEncoder) {
        this.inchesToEncoder = inchesToEncoder;
        this.wheelServoGearRatio = wheelServoGearRatio;
        this.widthOfRobot = widthOfRobot;
        this.lengthOfRobot = lengthOfRobot;
        this.servoMaxAngle = servoMaxAngle;
        this.turnAngle = Math.atan(widthOfRobot/lengthOfRobot);
    }

    public class SwerveWheel {
        public String name;
        public String servoConfigName;
        public Servo servo;
        public DcMotor motor;

        public double targetAngle = 0;
        public int modifier = 1;
        public double offset = 0;
        public double minWheelAngle=0;
        public double maxWheelAngle = 0;

        public SwerveWheel(double offset) {
          setOffset(offset);
        }

        public SwerveWheel (String name){
            this.name = name;
        }

        public void setOffset(double offset) {
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

        @Override
        public String toString() {
            return String.format("%s angle: %.2f, mod: %d, pos: %.2f, offset: %.2f encoder: %d", name, targetAngle, modifier, wheelAngleToServoPosition(), offset, motor.getCurrentPosition());
        }
    }

    public SwerveWheel frontLeftSwerveWheel = new SwerveWheel("FL");
    public SwerveWheel frontRightSwerveWheel = new SwerveWheel("FR");
    public SwerveWheel backLeftSwerveWheel = new SwerveWheel("BL");
    public SwerveWheel backRightSwerveWheel = new SwerveWheel("BR");

    public SwerveWheel[] swerveWheels = {frontLeftSwerveWheel, frontRightSwerveWheel, backLeftSwerveWheel, backRightSwerveWheel};

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

        setWheelMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftGrabber = hwMap.get(Servo.class, "leftGrabber");
        rightGrabber = hwMap.get(Servo.class, "rightGrabber");
        leftGrabber.setDirection(Servo.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize all servos
        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
        servoFrontRight = hardwareMap.servo.get("servoFrontRight");
        servoBackLeft = hardwareMap.servo.get("servoBackLeft");
        servoBackRight = hardwareMap.servo.get("servoBackRight");

        crane = hardwareMap.servo.get("crane");
        jaw = hardwareMap.servo.get("jaw");

        jaw.setDirection(Servo.Direction.REVERSE);

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

        readServoOffsets();

        frontLeftSwerveWheel.servo = servoFrontLeft;
        frontLeftSwerveWheel.motor = frontLeft;

        frontRightSwerveWheel.servo = servoFrontRight;
        frontRightSwerveWheel.motor = frontRight;

        backLeftSwerveWheel.servo = servoBackLeft;
        backLeftSwerveWheel.motor = backLeft;

        backRightSwerveWheel.servo = servoBackRight;
        backRightSwerveWheel.motor = backRight;
    }

    public void readServoOffsets() {
        try {
            offsets = FileUtilities.readDoubleConfigFile(CONFIG_FILENAME);
        } catch (IOException e) {
            offsets = new ArrayList<>();
        }

        for (int i = 0; i < swerveWheels.length; i++) {
            if (offsets.size() < i) {
                offsets.add(0.0);
            }
        }

        frontLeftSwerveWheel.setOffset(offsets.size() > 0 ? offsets.get(0) : 0.0);
        frontRightSwerveWheel.setOffset(offsets.size() > 1 ? offsets.get(1) : 0.0);
        backLeftSwerveWheel.setOffset(offsets.size() > 2 ? offsets.get(2) : 0.0);
        backRightSwerveWheel.setOffset(offsets.size() > 3 ? offsets.get(3) : 0.0);
    }

    public String writeOffsets() {
        try {
            FileUtilities.writeConfigFile(CONFIG_FILENAME, offsets);
        } catch (Exception e) {
            return String.format("Error writing to file. %s", e.getMessage());
        }
        return null;
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
        frontLeft.setTargetPosition(position* frontLeftSwerveWheel.modifier);
        frontRight.setTargetPosition(position* frontRightSwerveWheel.modifier);
        backLeft.setTargetPosition(position* backLeftSwerveWheel.modifier);
        backRight.setTargetPosition(position* backRightSwerveWheel.modifier);
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

    public void swerveStraightAbsolute(double joyWheelAngle, double power) {
        double absoluteAngle = joyWheelAngle - getAngle();
        swerveMove(absoluteAngle, absoluteAngle, absoluteAngle, absoluteAngle, power);
    }

    public void swerveTurn(double power) {

        double fLAngle = joystickPositionToWheelAngle(1, 1);
        double fRAngle = joystickPositionToWheelAngle(1, -1);
        double bLAngle = joystickPositionToWheelAngle(-1, 1);
        double bRAngle = joystickPositionToWheelAngle(-1, -1);

        swerveMove(fLAngle, fRAngle, bLAngle, bRAngle, power);
    }

    public void swerveMove(double fLAngle, double fRAngle, double bLAngle, double bRAngle, double power) {

        angleCheck(fLAngle, frontLeftSwerveWheel);
        angleCheck(fRAngle, frontRightSwerveWheel);
        angleCheck(bLAngle, backLeftSwerveWheel);
        angleCheck(bRAngle, backRightSwerveWheel);

        double servoPositionfL = frontLeftSwerveWheel.wheelAngleToServoPosition();
        double servoPositionfR = frontRightSwerveWheel.wheelAngleToServoPosition();
        double servoPositionbL = backLeftSwerveWheel.wheelAngleToServoPosition();
        double servoPositionbR = backRightSwerveWheel.wheelAngleToServoPosition();

        setWheelServoPosition(servoPositionfL, servoPositionfR, servoPositionbL, servoPositionbR);

        double frontLeftPower = frontLeftSwerveWheel.modifier * power;
        double frontRightPower = frontRightSwerveWheel.modifier * power;
        double backLeftPower = backLeftSwerveWheel.modifier * power;
        double backRightPower = backRightSwerveWheel.modifier * power;

        setWheelMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void moveLift (int counts){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setTargetPosition(counts);
        frontLeft.setPower(.3);
        while(frontLeft.isBusy());
        frontLeft.setPower(0);

    }

    public double getCurrentTurnPower(double absCurrent, double absGoal, double absStart, double maxPower) {
        double relCurrent = AngleUtilities.getNormalizedAngle(absCurrent - absStart);
        double relGoal = AngleUtilities.getNormalizedAngle(absGoal - absStart);
        double remainingDistance = AngleUtilities.getNormalizedAngle(relGoal - relCurrent);

        double basePower = basePowerRatio * remainingDistance;
        double stallPower = stallPowerRatio * Math.signum(remainingDistance);

        return Range.clip(basePower + stallPower, -Math.abs(maxPower), Math.abs(maxPower));
    }
}

