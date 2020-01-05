package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;

@TeleOp (name = "ImprovedGamepad TeleOp" )
public class TestGamepad extends OpMode {

    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;

    @Override
    public void init() {
        this.improvedGamepad1 = new ImprovedGamepad(gamepad1, timer, "g1_");
        this.improvedGamepad2 = new ImprovedGamepad(gamepad2, timer, "g2_");
    }

    @Override
    public void loop() {
        improvedGamepad1.update();
        improvedGamepad2.update();

        telemetry.addData("Pressed", improvedGamepad1.left_stick_y.isPressed());
        telemetry.addData("Pressed Count", improvedGamepad1.left_stick_y.getPressedCounts());
        telemetry.addData("Pressed Time", improvedGamepad1.left_stick_y.getPressedElapseTime());

        telemetry.addData("Raw x", improvedGamepad1.left_stick_x.getRawValue());
        telemetry.addData("Raw y", improvedGamepad1.left_stick_y.getRawValue());
        telemetry.addData("x", improvedGamepad1.left_stick_x.getValue());
        telemetry.addData("y", improvedGamepad1.left_stick_y.getValue());

        telemetry.addData("Raw Radius", improvedGamepad1.raw_left_stick_radius);
        telemetry.addData("Radius", improvedGamepad1.left_stick_radius);
        telemetry.addData("Angle", improvedGamepad1.left_stick_angle);

        telemetry.update();
    }
}
