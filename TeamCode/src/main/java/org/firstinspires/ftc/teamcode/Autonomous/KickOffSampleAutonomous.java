package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.KickOffSampleHardware;

@Autonomous (name = "Kick-off Sample Autonomous")
public class KickOffSampleAutonomous extends LinearOpMode {
    public KickOffSampleHardware robot = new KickOffSampleHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
    }
}
