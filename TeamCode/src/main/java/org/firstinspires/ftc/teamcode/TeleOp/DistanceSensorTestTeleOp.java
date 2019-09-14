package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "DistanceSensorTest")
public class DistanceSensorTestTeleOp extends OpMode {
DcMotor leftDrive;
DcMotor rightDrive;
DistanceSensor distanceSensor;

    @Override
    public void init() {
      leftDrive = this.hardwareMap.dcMotor.get("left_drive");
      rightDrive = this.hardwareMap.dcMotor.get("right_drive");
      distanceSensor = this.hardwareMap.get (DistanceSensor.class,"distance");
    }

    @Override
    public void loop() {

    }
}
