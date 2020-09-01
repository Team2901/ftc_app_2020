package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KickOffSampleHardware {

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
