package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorAndServoTester", group = "Test")
public class MotorAndServoTester extends OpMode {
    Servo mrServo;

    DcMotor motor;

    @Override
    public void init() {
        mrServo = hardwareMap.servo.get("servo");
        motor = this.hardwareMap.dcMotor.get("motor");

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mrServo.setPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.x){
            mrServo.setPosition(0);
        }
        if (gamepad1.a){
            mrServo.setPosition(1.0/3);
        }
        if (gamepad1.b){
            mrServo.setPosition(2.0/3);
        }
        if (gamepad1.y){
            mrServo.setPosition(1.0);

        }

        float leftStickY = -gamepad1.left_stick_y;

        motor.setPower(leftStickY);

        telemetry.addData("Servo position",mrServo.getPosition());
        telemetry.addData("Motor power",motor.getPower());
        telemetry.addData("Motor ticks",motor.getCurrentPosition());

        telemetry.addData("Press X","goes to position 0");
        telemetry.addData("Press A","goes to position 0.33");
        telemetry.addData("Press B","goes to position 0.66");
        telemetry.addData("Press Y","goes to position 1.0");
        telemetry.addData("Left Joystick Y","sets motor power");
        telemetry.update();

    }
}
