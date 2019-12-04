package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

import java.util.ArrayList;

@TeleOp(name = "HardwareTester")
public class HardwareTester extends OpMode {

    SkystoneHardware robot = new SkystoneHardware();

    DcMotor  motor1;
    Servo servo1;

    String[] motorNames = {"frontLeft","frontRight","backLeft","backRight","lift"};
    String[] servoNames = {"servoFrontLeft","servoFrontRight","servoBackLeft","servoBackRight"};

    int motorIndex;

    Servo[] servos = {robot.servoFrontLeft,robot.servoFrontRight,robot.servoBackLeft,robot.servoBackRight};
    int servoIndex;

    ArrayList<DcMotor> motorArrayList;
    ArrayList<Servo>servoArrayList;

    @Override
    public void init() {
        robot.init(hardwareMap);
        for(int i = 0; i < motorNames.length; i++){
            String motorName = motorNames[motorIndex];
            DcMotor motor = hardwareMap.dcMotor.get(motorName);
            motorArrayList.add(motor);
        }
        for(int i = 0; i < servoNames.length; i++){
            String servoName = servoNames[servoIndex];
            Servo servo = hardwareMap.servo.get(servoName);
            servoArrayList.add(servo);
        }
    }

    @Override
    public void loop() {

        if(this.gamepad1.dpad_up){
            motorIndex++;
            if(motorIndex >= motorArrayList.size()){
                motorIndex = 0;
            }
        } else if(this.gamepad1.dpad_down) {
            motorIndex--;
            if(motorIndex < 0){
                motorIndex = motorArrayList.size() - 1;
            }
        }
        motor1 = motorArrayList.get(motorIndex);

        if(this.gamepad1.dpad_right){
            servoIndex++;
            if(servoIndex >= servoArrayList.size()){
                servoIndex = 0;
            }
        } else if(this.gamepad1.dpad_left) {
            servoIndex--;
            if(servoIndex < 0){
                servoIndex = servoArrayList.size() - 1;
            }
        }

        servo1 = servoArrayList.get(servoIndex);

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
            telemetry.addData("Motor name", motorNames[motorIndex]);
        }

        telemetry.addData("D Pad Up/Down", "Increment/decrement motors");

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
            telemetry.addData("Servo name",servoNames[servoIndex]);
        }

        telemetry.addData("D Pad Right/Left", "Increment/decrement servos");


        telemetry.update();

    }
}
