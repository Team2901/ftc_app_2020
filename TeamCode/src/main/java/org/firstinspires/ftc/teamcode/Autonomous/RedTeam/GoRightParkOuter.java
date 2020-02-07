package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

//@Autonomous(name = "Sky_Red_Go_Right_Park_Outer", group = "_RED")
public class GoRightParkOuter extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        this.moveInches(0.0,2, .2);
        this.moveInches(-90,24, .2);


    }
}
