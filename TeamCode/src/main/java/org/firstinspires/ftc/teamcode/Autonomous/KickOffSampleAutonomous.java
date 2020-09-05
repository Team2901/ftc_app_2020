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
    public void moveInches (double inches){
        int ticks = (int) (inches * robot.TICKS_PER_INCH);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + ticks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + ticks);
    }
}
