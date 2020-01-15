package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester", group = "Test")
public class ServoTester extends OpMode {
    private Servo mrServo;
    private boolean isLastRightBumperPressed;
    private boolean isLastLeftBumperPressed;
    private boolean isLastRightTriggerPressed;
    private boolean isLastLeftTriggerPressed;

    @Override
    public void init() {
        mrServo = hardwareMap.servo.get("servo");
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
        if (gamepad1.right_bumper && !isLastRightBumperPressed){
            double newServoPosition = .05 + mrServo.getPosition();
            mrServo.setPosition(newServoPosition);
        }
        if (gamepad1.left_bumper && !isLastLeftBumperPressed) {
            double newServoPosition = -.05 + mrServo.getPosition();
            mrServo.setPosition(newServoPosition);
        }
        if (gamepad1.right_trigger > 0.01 && !isLastRightTriggerPressed){
            double newServoPosition = .01 + mrServo.getPosition();
            mrServo.setPosition(newServoPosition);
        }
        if (gamepad1.left_trigger > 0.01 && !isLastLeftTriggerPressed) {
            double newServoPosition = -.01 + mrServo.getPosition();
            mrServo.setPosition(newServoPosition);
        }

        isLastRightBumperPressed = gamepad1.right_bumper;
        isLastLeftBumperPressed = gamepad1.left_bumper;
        isLastRightTriggerPressed = gamepad1.right_trigger > 0.01;
        isLastLeftTriggerPressed = gamepad1.left_trigger > 0.01;
        telemetry.addData("CurrentPosition",mrServo.getPosition());
        telemetry.addData("Press X","goes to position 0");
        telemetry.addData("Press A","goes to position 0.33");
        telemetry.addData("Press B","goes to position 0.66");
        telemetry.addData("Press Y","goes to position 1.0");

    }
}
