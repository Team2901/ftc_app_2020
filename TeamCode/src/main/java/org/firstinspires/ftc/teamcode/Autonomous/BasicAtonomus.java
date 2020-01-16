package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name= "Basic Autonmus")
public class BasicAtonomus extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        // fowards 2ft ,turn 90 counterclockwise, go forwards, turn 70
        moveInches(0,12 ,0.1);
        turnTo(90);
        robot.swerveStraight(0,0);
        //Do swerve turn after turn
        moveInches(0,120,0.2);
       // turnTo(70);
        while (opModeIsActive())
        {

        }
    }
}
