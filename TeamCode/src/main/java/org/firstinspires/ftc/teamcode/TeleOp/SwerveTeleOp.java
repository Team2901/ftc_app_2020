package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

@TeleOp (name = "Swerve Teleop")
public class SwerveTeleOp extends OpMode {


    private boolean isLastRightBumperPressed;
    private boolean isLastLeftBumperPressed;
    private boolean isLastRightTriggerPressed;
    private boolean isLastLeftTriggerPressed;



    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        //The y position is -1 to correct the joystick directions
        robot.setDriveServoPosition(0);


    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        //lift motors below


        //swerve code below
        double radius = Math.sqrt(Math.pow(joystickPositionX, 2) + Math.pow(joystickPositionY, 2));

        Double wheelAngle = null;


        if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
            robot.swerveTurn(gamepad1.right_stick_x , gamepad1.left_bumper);
        } else if (radius > .2) {
            wheelAngle = robot.joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            robot.setPower(joystickPositionX, joystickPositionY, 1 , gamepad1.left_bumper);
        } else {
            robot.setPower(0, 0, 1 ,gamepad1.left_bumper );
        }
/*
        if (gamepad2.y) {
            wheelAngle = 0d;
        } else if (gamepad2.x) {
            wheelAngle = 90d;
        } else if (gamepad2.a) {
            wheelAngle = 180d;
        } else if (gamepad2.b) {
            wheelAngle = 270d;
        } else if (gamepad2.right_bumper && !isLastRightBumperPressed){
            wheelAngle = robot.currentAngle + 90;
        }
        if (gamepad2.left_bumper && !isLastLeftBumperPressed) {
            wheelAngle = robot.currentAngle - 90;
        }
        if (gamepad2.right_trigger > 0.01 && !isLastRightTriggerPressed){
            wheelAngle = robot.currentAngle + 5;
        }
        if (gamepad2.left_trigger > 0.01 && !isLastLeftTriggerPressed) {
            wheelAngle = robot.currentAngle - 5;
        }
*/
        if (wheelAngle != null) {
            robot.setDriveServoPosition(wheelAngle);
        }
/*
        isLastRightBumperPressed = gamepad2.right_bumper;
        isLastLeftBumperPressed = gamepad2.left_bumper;
        isLastRightTriggerPressed = gamepad2.right_trigger > 0.01;
        isLastLeftTriggerPressed = gamepad2.left_trigger > 0.01;
*/


        telemetry.update();
    }


}