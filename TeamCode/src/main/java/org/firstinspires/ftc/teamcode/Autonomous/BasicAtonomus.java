package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;

@Autonomous(name= "Basic Autonmus")
public class BasicAtonomus extends LinearOpMode {
    public BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    public ToolBox toolBox = new ToolBox(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
    }
}
