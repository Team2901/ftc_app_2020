package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Red Quarry Park Wall (5pt)", group = "_RED")
public class RedQuarryParkWall extends RedSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null, true, 2);
        waitForStart();
        this.park(SAFE_WALL_DISTANCE_INCHES,-90);
    }
}
