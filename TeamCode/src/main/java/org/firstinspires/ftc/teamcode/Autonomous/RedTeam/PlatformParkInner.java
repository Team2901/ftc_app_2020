package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.ToolBox;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

@Autonomous(name = "Sky_Red_Platform_Park_Inner")



public class PlatformParkInner extends LinearOpMode {

    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ToolBox toolBox = new ToolBox(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        toolBox.platformParkInner(Color.RED);
    }
}
