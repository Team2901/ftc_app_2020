package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;



@Autonomous(name = "New", group = "BlueTeam")
public class NewAutonomous extends BaseSkyStoneAuto {


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.crane.setPosition(.033);
        robot.jaw.setPosition(robot.OPEN_JAW);
        this.moveInches(0,31, .4);
        robot.jaw.setPosition(robot.CLOSED_JAW);
        robot.lift.setTargetPosition(-103);
        this.moveInches(0,31, -0.4);

    }

}
