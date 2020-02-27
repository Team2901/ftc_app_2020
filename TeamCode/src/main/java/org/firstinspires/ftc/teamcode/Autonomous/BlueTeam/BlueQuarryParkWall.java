package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;


@Autonomous(name = "Blue Quarry Park Wall", group = "_BLUE")
public class BlueQuarryParkWall extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        this.park(SAFE_WALL_DISTANCE_INCHES,90);
    }
}
