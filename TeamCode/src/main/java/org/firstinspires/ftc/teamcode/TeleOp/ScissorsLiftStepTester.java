package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ScissorsLiftStepTester")
public class ScissorsLiftStepTester extends OpMode {
    private boolean isYPressed;
    private boolean isXPressed;
    private boolean isBPressed;
    private boolean isAPressed;
    @Override
    public void init() {}
    int step = 0;
    int topStep = 0;
    @Override
    public void loop() {
        if(this.gamepad2.y && !isYPressed){
            step = step+1;
        }
        if(this.gamepad2.a && !isAPressed){
            step = step-1;
            if(step < 0){
                step = 0;
            }
        }

        if(this.gamepad2.b && !isBPressed){
            topStep = step;
            step = 0;
        }

        if(this.gamepad2.x && !isXPressed){
            step = topStep;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Top Step", topStep);
        telemetry.update();

        isYPressed = gamepad2.y;
        isXPressed = gamepad2.x;
        isBPressed = gamepad2.b;
        isAPressed = gamepad2.a;
    }
}
