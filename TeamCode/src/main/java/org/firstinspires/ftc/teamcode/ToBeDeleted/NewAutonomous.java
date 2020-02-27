package org.firstinspires.ftc.teamcode.ToBeDeleted;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;



@Autonomous(name = "New 1", group = "BlueTeam")
public class NewAutonomous extends BaseSkyStoneAuto {


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.speak("FUCK","en","US");


        robot.crane.setPosition(0);

        waitForStart();

        robot.crane.setPosition(1);
        robot.jaw.setPosition(1);

        this.moveInches(0,28, .3);
        robot.jaw.setPosition(0);
        robot.lift.setTargetPosition(-103);
        this.moveInches(0,-24, 0.4);

        turnTo(90, .2);


        this.moveInches(0, 98, 0.3);
        turnTo(-90, .2);
        robot.lift.setTargetPosition(-206);



        while(opModeIsActive());
    }

}
