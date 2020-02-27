package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;


@Autonomous(name = "Red Quarry Park Wall", group = "_RED")
public class RedQuarryParkWall extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false);
        waitForStart();
        this.park(SAFE_WALL_DISTANCE_INCHES,-90);
    }
}
