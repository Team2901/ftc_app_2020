package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.RedSkyStoneAuto;

@Autonomous (name = "Red Foundation Park Wall (5pt)", group = "_RED")
public class RedFoundationParkWall extends RedSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null);
        waitForStart();
        this.park(SAFE_WALL_DISTANCE_INCHES,90);
    }

}
