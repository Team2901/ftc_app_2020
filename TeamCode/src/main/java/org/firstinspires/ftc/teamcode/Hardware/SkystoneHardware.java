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

public class SkystoneHardware {

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
    public final static double WHEEL_MIN_ANGLE = 0;
    public final static double WHEEL_MAX_ANGLE = SERVO_MAX_ANGLE * WHEEL_SERVO_GEAR_RATIO;

    public class SwerveWheel {
        public double targetAngle = 0;
        public int modifier = 1;
        public double offset = 0;

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
        public SwerveWheel frontLeftMotor = new SwerveWheel(FRONT_LEFT_OFFSET);
        public SwerveWheel frontRightMotor = new SwerveWheel(FRONT_RIGHT_OFFSET);
        public SwerveWheel backLeftMotor = new SwerveWheel(BACK_LEFT_OFFSET);
        public SwerveWheel backRightMotor = new SwerveWheel(BACK_RIGHT_OFFSET);
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

    public Servo bridgeTickler;
    public Servo crane;
    public Servo jaw;
    public Servo wrist;

    public SwerveWheels swerveWheels = new SwerveWheels();

    //Sensors and Things
    public BNO055IMU imu;
    public IntegratingGyroscope gyroscope;

    public double offset = 0;

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

    public double getRawAngle() {
        Orientation orientation = gyroscope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUtilities.getNormalizedAngle(orientation.firstAngle);
    }

    public double getAngle() {
        return AngleUtilities.getNormalizedAngle(getRawAngle() + offset);
    }

    public void setWheelMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        backRight.setPower(-backRightPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(-frontLeftPower);
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

    public boolean wheelsAreBusy() {
        return frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy();

    }

    public void wait(int milliseconds, LinearOpMode opMode) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.milliseconds() < milliseconds) {

        }
    }
}

