package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

public class SkystoneHardware {

    public HardwareMap hardwareMap;

    //Made for a 4 wheel swerve drive system
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

   //Steering servo for their respective motor
    public Servo servoFrontLeft;
    public Servo servoFrontRight;
    public Servo servoBackLeft;
    public Servo servoBackRight;

    //Sensors and Things
    public BNO055IMU imu;
    public IntegratingGyroscope gyroscope;

    public void init(HardwareMap hwMap){
        hardwareMap = hwMap;

        //Inititialize all Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        //Initialize all servos
        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
        servoFrontRight =  hardwareMap.servo.get("servoFrontRight");
        servoBackLeft =  hardwareMap.servo.get("servoBackLeft");
        servoBackRight =  hardwareMap.servo.get("servoBackRight");

        setAllSteeringServos(0);

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

    public void setAllSteeringServos(double position){
        servoBackRight.setPosition(position);
        servoBackLeft.setPosition(position);
        servoFrontLeft.setPosition(position);
        servoFrontRight.setPosition(position);


    }
}

