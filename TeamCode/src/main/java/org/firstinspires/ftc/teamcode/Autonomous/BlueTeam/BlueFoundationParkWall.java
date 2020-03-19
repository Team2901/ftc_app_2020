package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

@Autonomous (name = "Blue Foundation Park Wall (5pt)", group = "_BLUE")
public class BlueFoundationParkWall extends BlueSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null);
        waitForStart();
        this.park(SAFE_WALL_DISTANCE_INCHES,-90);
    }
}
