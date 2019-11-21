package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

public class JoystickTestProgram extends OpMode {

    final SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        /*
        Is this button pressed? If this button is pressed buttonBlink is pressed. The end goal
        is to print that the button has been pressed (this is to be done for one controller).
         */
        if(gamepad1.a){
            telemetry.addLine("button a is pressed");
        }
        if(gamepad1.b){
            telemetry.addLine("button b is pressed");
        }
        if(gamepad1.x){
            telemetry.addLine("button x is pressed");
        }
        if(gamepad1.y){
            telemetry.addLine("button y is pressed");
        }
        if(gamepad1.left_bumper==true){
            telemetry.addLine("left bumper is pressed");
        }
        if(gamepad1.right_bumper==true){
            telemetry.addLine("right bumper is pressed");
        }
        if(gamepad1.left_trigger>.25){
            telemetry.addLine("left trigger is pressed");
        }
        if(gamepad1.right_trigger >.25){
            telemetry.addLine("right trigger is pressed");
        }
        if(gamepad1.left_stick_x>0){
            telemetry.addLine("left joystick is pointed right");
        }
        if(gamepad1.left_stick_x<0){
            telemetry.addLine("left joystick is pointed left");
        }
        if(gamepad1.right_stick_x>0){
            telemetry.addLine("right joystick is pointed right");
        }
        if(gamepad1.right_stick_x<0){
            telemetry.addLine("right joystick is pointed left");
        }
        if(gamepad1.left_stick_y>0){
            telemetry.addLine("left joystick is pointed down");
        }
        if(gamepad1.left_stick_y<0){
            telemetry.addLine("left joystick is pointed up");
        }
        if(gamepad1.right_stick_y>0){
            telemetry.addLine("right joystick is pointed down");
        }
        if(gamepad1.right_stick_y<0){
            telemetry.addLine("right joystick is pointed up");
        }
        telemetry.update();
    }
}
