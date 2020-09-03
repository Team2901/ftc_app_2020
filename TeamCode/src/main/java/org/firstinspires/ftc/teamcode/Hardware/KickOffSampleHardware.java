package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KickOffSampleHardware {

    /*
    This is the math that is used for encoders. The ticks per motor rev is
    how many encoder ticks it takes for the motor to turn once
     */
    public static final double TICKS_PER_MOTOR_REV = 1140;
    //This is the ratio of the gears.
    public static final double DRIVE_GEAR_RATIO = 1;
    /*
    To find the number of encoder ticks it takes to make the wheel go around once,
    you need to multiply the gear ratio by the motor tick count
     */
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    /*
    To find the wheel circumference, you need to multiply the wheel diameter by pi.
     */
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    /*
    This last one calculates how many encoder ticks it takes to go one inch.
    It divides the ticks per wheel rotation by the wheel circumference.
     */
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;

    //Define the left and right motors as class variables
    public DcMotor leftDrive;
    public DcMotor rightDrive;

    public void init(HardwareMap hardwareMap){

        //initializing the left and right motors
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        //setting motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //stopping motors during initialization
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        //setting motors up to run with encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
