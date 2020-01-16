package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.ToolBox;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;

@Autonomous(name = "Sky_Blue_Platform_Park_Inner", group = "_BLUE")



public class PlatformParkInner extends LinearOpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ToolBox toolBox = new ToolBox(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        toolBox.platformParkInner(Color.BLUE);
    }
}
