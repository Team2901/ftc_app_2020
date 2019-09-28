package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo_testing")
public class Servo_testing extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Servo handServo;
        Servo wristServo;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //wrist servo goes up and down hand servo goes left to right.
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.left_trigger > 0) {
                handServo.setPosition(0.0);
            } else if (gamepad1.right_trigger > 0) {
                handServo.setPosition(1.0);
            }
            if (gamepad2.left_trigger > 0) {
                wristServo.setPosition(0.0);
            } else if (gamepad2.right_trigger > 0) {

                wristServo.setPosition(1.0);
            }

            telemetry.update();

        }
    }
}

