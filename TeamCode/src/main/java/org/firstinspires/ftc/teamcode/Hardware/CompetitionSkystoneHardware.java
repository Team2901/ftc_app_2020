package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CompetitionSkystoneHardware extends BaseSkyStoneHardware{

    public static final double WHEEL_POWER_RATIO = .3;


    public final static double WHEEL_SERVO_GEAR_RATIO = 84.0/60.0;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan(WIDTH_OF_ROBOT/ LENGTH_OF_ROBOT);
    public final static int SERVO_MAX_ANGLE = 280;
    public final static double INCHES_TO_ENCODER = 13;

    public CompetitionSkystoneHardware(){
        super (WIDTH_OF_ROBOT,
                LENGTH_OF_ROBOT,
                WHEEL_SERVO_GEAR_RATIO,
                SERVO_MAX_ANGLE,
                INCHES_TO_ENCODER);
    }


    public void init(HardwareMap hwMap) {

        super.init(hwMap);
        servoFrontLeft.setDirection(Servo.Direction.REVERSE);
        servoFrontRight.setDirection(Servo.Direction.REVERSE);
        servoBackLeft.setDirection(Servo.Direction.REVERSE);
        servoBackRight.setDirection(Servo.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Platform Grabber
        //setGrabberPositition(.7,.84);
    }
}

