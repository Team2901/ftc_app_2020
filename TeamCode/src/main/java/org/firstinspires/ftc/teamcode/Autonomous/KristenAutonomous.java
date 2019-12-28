package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous (name = "Kristen V2 Autonomous")
public class KristenAutonomous extends LinearOpMode {

    //DcMotor leftMotor;
    //DcMotor rightMotor;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public static final double LEFT_GRABBER_MIN = 0.75;
    public static final double RIGHT_GRABBER_MAX = 0.25;
    public static final double LEFT_GRABBER_MAX = 0.25;
    public static final double RIGHT_GRABBER_MIN =0.75;
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI;
    public static final double GEAR_RATIO =  2;
    public static final double ENCODER_COUNTS_PER_MOTOR_REV = 1960 ;

    public int  getTargetEncoderCounts(int distanceInches){
        return (int)(distanceInches/WHEEL_CIRCUMFERENCE * GEAR_RATIO * ENCODER_COUNTS_PER_MOTOR_REV);
        }


    @Override
    public void runOpMode() throws InterruptedException {


        //leftMotor = hardwareMap.dcMotor.get("left_drive");
        //leftMotor = hardwareMap.get(DcMotor.class, "left_drive");

        //rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftGrabber = hardwareMap.get(Servo.class, "Left_grabber");
        rightGrabber = hardwareMap.get(Servo.class, "Right_grabber");

        rightGrabber.setPosition (RIGHT_GRABBER_MIN);
        leftGrabber.setPosition (LEFT_GRABBER_MAX);

        /*leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int target = (getTargetEncoderCounts(31));
        telemetry.addData("Target:" , target);
        telemetry.update();

        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
       */
        waitForStart();


        //leftMotor.setPower(0.5);
        //rightMotor.setPower(0.5);

        /*while  (leftMotor.isBusy()){

            telemetry.addData("Count:",leftMotor.getCurrentPosition() );
            telemetry.update();
            idle();

        }
       */
        rightGrabber.setPosition (RIGHT_GRABBER_MAX);
        leftGrabber.setPosition (LEFT_GRABBER_MIN);

        /*leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);

        leftMotor.setPower(.25);
        rightMotor.setPower(.25);

        while (leftMotor.isBusy()){

            idle();

        }
       */
    }

}


