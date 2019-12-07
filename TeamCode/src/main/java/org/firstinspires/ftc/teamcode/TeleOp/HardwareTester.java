package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

import java.util.ArrayList;

@TeleOp(name = "HardwareTester2")
public class HardwareTester extends OpMode {

    SkystoneHardware robot = new SkystoneHardware();

    DcMotor  motorUnderTest;
    Servo servoUnderTest;

    String[] motorNames = {"frontLeft","frontRight","backLeft","backRight","lift"};
    String[] servoNames = {"servoFrontLeft","servoFrontRight","servoBackLeft","servoBackRight"};

    int motorIndex;

    Servo[] servos = {robot.servoFrontLeft,robot.servoFrontRight,robot.servoBackLeft,robot.servoBackRight};
    int servoIndex;

    ArrayList<DcMotor> motorArrayList = new ArrayList<>();
    ArrayList<Servo> servoArrayList = new ArrayList<>();

    @Override
    public void init() {
        robot.init(hardwareMap);
        for(int i = 0; i < motorNames.length; i++){
            String motorName = motorNames[i];
            DcMotor motor = hardwareMap.dcMotor.get(motorName);
            motorArrayList.add(motor);
            telemetry.addData("Motor"+i,motorName,motor);
        }
        for(int i = 0; i < servoNames.length; i++){
            String servoName = servoNames[i];
            Servo servo = hardwareMap.servo.get(servoName);
            servoArrayList.add(servo);
            telemetry.addData("Servo"+i,servoArrayList.get(i));
        }

        telemetry.update();
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
        motorUnderTest = motorArrayList.get(motorIndex);

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

        servoUnderTest = servoArrayList.get(servoIndex);

        if(motorUnderTest != null){
            if(Math.abs(this.gamepad1.left_stick_x) > 0.25 ){
                motorUnderTest.setPower(this.gamepad1.left_stick_x);
            } else {
                motorUnderTest.setPower(0);
            }

            if(this.gamepad1.a){
                motorUnderTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorUnderTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("Left X Stick", "Set Power");
            telemetry.addData("A","Resetting encoders");
            telemetry.addData("Current position", motorUnderTest.getCurrentPosition());
            telemetry.addData("Power", motorUnderTest.getPower());
            telemetry.addData("Motor name", motorNames[motorIndex]);
            telemetry.addData("Motor object",motorUnderTest);
        }

        telemetry.addData("D Pad Up/Down", "Increment/decrement motors");

        if(servoUnderTest != null){
            if(this.gamepad1.left_bumper){
                servoUnderTest.setPosition(servoUnderTest.getPosition()-0.1);
            } else if(this.gamepad1.right_bumper){
                servoUnderTest.setPosition(servoUnderTest.getPosition()+0.1);
            } else if(this.gamepad1.left_trigger > 0.25){
                servoUnderTest.setPosition(servoUnderTest.getPosition()-0.2);
            } else if(this.gamepad1.right_trigger > 0.25) {
                servoUnderTest.setPosition(servoUnderTest.getPosition()+0.2);
            }

            telemetry.addData("Left bumper","-0.1");
            telemetry.addData("Right bumper","+0.1");
            telemetry.addData("Left trigger","-0.2");
            telemetry.addData("Right trigger","+0.2");
            telemetry.addData("Position", servoUnderTest.getPosition());
            telemetry.addData("Servo name",servoNames[servoIndex]);
        }

        telemetry.addData("D Pad Right/Left", "Increment/decrement servos");


        telemetry.update();

    }
}
