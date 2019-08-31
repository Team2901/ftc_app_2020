package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Motor Encoder")
public class TestEncoder extends OpMode {

    public DcMotor motor;


    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        motor.setPower( - gamepad1.right_stick_y);

        telemetry.addData("Joystick" , gamepad1.right_stick_y);
        telemetry.addData("MotorPower" , motor.getPower());
        telemetry.addData("Motor Position" , motor.getCurrentPosition());
        telemetry.addData("Ticks per Revolution" , motor.getMotorType().getTicksPerRev());
        telemetry.update();
    }
}