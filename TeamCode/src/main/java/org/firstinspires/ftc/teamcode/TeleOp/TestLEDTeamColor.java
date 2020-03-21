package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.CompetitionSkystoneHardware;

@TeleOp(name = "Test LED Team Color", group = "TEST")
public class TestLEDTeamColor extends OpMode {

    CompetitionSkystoneHardware robot = new CompetitionSkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
