package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Tri Servo Arm Test")
public class TriServoArmTest extends OpMode {

    public Servo winchServo;
    public Servo rotatingServo;
    public Servo pinchServo;

    @Override
    public void init() {
        winchServo = hardwareMap.servo.get("winchServo");
        rotatingServo = hardwareMap.servo.get("rotatingServo");
        pinchServo = hardwareMap.servo.get("pinchServo");
        rotatingServo.setPosition(0);
        pinchServo.setPosition(0);
    }

    @Override
    public void loop() {
        double winchPosition = winchServo.getPosition();
        double leftStickY = -gamepad1.left_stick_y;
        if (leftStickY > 0.1) {
            winchServo.setPosition(winchPosition + 0.01);
        } else if (leftStickY < -0.1) {
            winchServo.setPosition(winchPosition - 0.01);
        }
        double positionRotatingServo = rotatingServo.getPosition();
        if (gamepad1.a){
            rotatingServo.setPosition(positionRotatingServo + 0.1);
        } else if (gamepad1.b){
            rotatingServo.setPosition(positionRotatingServo - 0.1);
        }
        double positionpinchServo = pinchServo.getPosition();
        if (gamepad1.right_trigger > 0.1) {
            pinchServo.setPosition(positionpinchServo + 0.1);
        }else if (gamepad1.left_trigger > 0.1){
            pinchServo.setPosition((positionpinchServo - 0.1));
        }
        telemetry.addData("winchServo", winchPosition);
        telemetry.addData("rotatingServo", positionRotatingServo);
        telemetry.addData("pinchServo", positionpinchServo);
        telemetry.update();
    }
}
