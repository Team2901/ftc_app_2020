package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name= "BluePlatform")
public class Platform extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        //
        moveInches(0, -12, 0.4);

        robot.swerveStraight(0, 0);
        //Do swerve turn after turn
        moveInches(90, -34, 0.4);

        robot.setGrabberPositition(0, 0);

        //turnTo(90);
        turnTo(70);
        while (opModeIsActive()) {

        }
    }
}
