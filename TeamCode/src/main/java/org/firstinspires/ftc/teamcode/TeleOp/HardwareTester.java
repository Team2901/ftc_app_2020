package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.TeleOp.SkystoneTeleOp;

public class HardwareTester extends OpMode {

    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        /*
        which buttons control what?
         */
        if(gamepad1.y){
            robot.frontLeft.setPower(.75);
        }
        if(gamepad1.b){
            robot.frontRight.setPower(.75);
        }
        if(gamepad1.x){
            robot.backLeft.setPower(.75);
        }
        if(gamepad1.a){
            robot.backRight.setPower(.75);
        }
        //Swerve servos
        if(gamepad1.left_bumper && gamepad1.y)
        {
            robot.servoFrontLeft.setPosition(1);
        }
        if(gamepad1.left_bumper && gamepad1.b){
            robot.servoFrontRight.setPosition(1);
        }
        if(gamepad1.left_bumper && gamepad1.x){
            robot.servoBackLeft.setPosition(1);
        }
        if(gamepad1.left_bumper && gamepad1.a){
            robot.servoBackRight.setPosition(1);
        }
        //other servos
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            robot. bridgeTickler.setPosition(1);
        }
        if(gamepad1.x && gamepad1.y){
            robot.wrist.setPosition(1);
        }
       // if(gamepad1.left_stick_x>0 && gamepad1.right_stick_x<0){
        //this should set the claw servo position preferable to the closed position so that the
        // joystick positions make more sense

        //}

    }
}
