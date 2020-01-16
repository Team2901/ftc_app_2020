package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name= "Basic Autonmus")
public class BasicAtonomus extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        // fowards 2ft ,turn 90 counterclockwise, go forwards, turn 70
        moveInches(0,24,0.4);
        turnTo(90);
        moveInches(0,12,0.4);
        turnTo(70);
    }
}
