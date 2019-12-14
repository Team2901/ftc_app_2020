package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;


@TeleOp(name="ScissorsLiftStepTester")
public class ScissorsLiftStepTester extends OpMode {
    private boolean isYPressed;
    private boolean isXPressed;
    private boolean isBPressed;
    private boolean isAPressed;
    double power=0;
    int step = 0;
    int topStep = 0;

    SkystoneHardware robot = new SkystoneHardware();


    @Override
    public void init() {
        robot.init(hardwareMap);
        DcMotor lift = this.hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    @Override
    public void loop() {
        
        if(this.gamepad2.y && !isYPressed){
            step = step+1;
            power= .5;
            //up by one
        }
        if(this.gamepad2.a && !isAPressed){
            step = step-1;
            if(step < 0){
                step = 0;
            }
            power= -.5;
            //down by one
        }

        if(this.gamepad2.b && !isBPressed){
            topStep = step;
            step = 0;
            power=.7;
            //to bottom
        }

        if(this.gamepad2.x && !isXPressed){
            step = topStep;
            power= -.7;
            //to top
        }
        int targetPosition = step*500;

        robot.lift.setTargetPosition(targetPosition);
        robot.lift.setPower(power);

        telemetry.addData("Step", step);
        telemetry.addData("Top Step", topStep);
        telemetry.update();

        isYPressed = gamepad2.y;
        isXPressed = gamepad2.x;
        isBPressed = gamepad2.b;
        isAPressed = gamepad2.a;
    }
    public int stepByOne (int step)
    {
        if(gamepad1.left_bumper=true && gamepad1.y)
        {
            step = step+1;

        }
        if (gamepad1.right_bumper=true && gamepad1.a){
            step = step-1;
            if(step < 0){
                step = 0;
            }
        }
        telemetry.addLine("level: " + step);
        return step;
    }

}
