package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Gamepad TeleOp" )
public class TestGamepad extends OpMode {
    ButtonState aButton = new ButtonState();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {

    }

    @Override
    public void loop() {
    double time = timer.time();
        aButton.update(gamepad1.a , time);

        telemetry.addData("Gamepad1.a", gamepad1.a);
        telemetry.addData("aButton State", aButton.state);
        telemetry.addData("aButton event Time", aButton.eventTime);
        telemetry.addData("Elsapsed Time", time);
        telemetry.addData("aButton name", aButton.name);
        telemetry.addData("is pressed" , aButton.onPressed(time));
        telemetry.update();

    }

    public class ButtonState {
        boolean state;
        double eventTime;
        String name = "NULL";

        public void update(boolean newState, double time) {
            if (state != newState) {
                eventTime = time;
            }
            
            state = gamepad1.a;
        }
        public boolean onPressed(double time) {
            if (eventTime == time && state){
                return true;
            }
            return false;
        }

    }


}
