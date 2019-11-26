package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.TeleOp.SkystoneTeleOp;
@TeleOp(name = "HardwareTester")
public class HardwareTester extends OpMode {

    SkystoneHardware robot = new SkystoneHardware();

    DcMotor  motor1;
    Servo servo1;

    DcMotor[] motors = {robot.frontLeft,robot.frontRight,robot.backLeft,robot.backRight,robot.lift};
    int motorindex;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        if(this.gamepad1.dpad_up){
            motorindex ++;
            if(motorindex >= motors.length){
                motorindex = 0;
            }
        } else if(this.gamepad1.dpad_down) {
            motorindex --;
            if(motorindex < 0){
                motorindex = motors.length - 1;
            }
        }
        motor1 = robot.frontLeft;
        servo1 = robot.servoFrontLeft;

        if(motor1 != null){
            if(Math.abs(this.gamepad1.left_stick_x) > 0.25 ){
                motor1.setPower(this.gamepad1.left_stick_x);
            } else {
                motor1.setPower(0);
            }

            if(this.gamepad1.a){
                motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("Left X Stick", "Set Power");
            telemetry.addData("A","Resetting encoders");
            telemetry.addData("Current position", motor1.getCurrentPosition());
            telemetry.addData("Power", motor1.getPower());
        }

        if(servo1 != null){
            if(this.gamepad1.left_bumper){
                servo1.setPosition(servo1.getPosition()+0.1);
            } else if(this.gamepad1.right_bumper){
                servo1.setPosition(servo1.getPosition()-0.1);
            } else if(this.gamepad1.left_trigger > 0.25){
                servo1.setPosition(servo1.getPosition()+0.2);
            } else if(this.gamepad1.right_trigger > 0.25) {
                servo1.setPosition(servo1.getPosition() - 0.2);
            }

            telemetry.addData("Left bumper","+0.1");
            telemetry.addData("Right bumper","-0.1");
            telemetry.addData("Left trigger","+0.2");
            telemetry.addData("Right trigger","-0.2");
            telemetry.addData("Position", servo1.getPosition());
        }


        telemetry.update();

    }
}
