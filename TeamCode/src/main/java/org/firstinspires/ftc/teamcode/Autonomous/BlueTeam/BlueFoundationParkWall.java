package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous (name = "Blue Foundation Park Wall", group = "_BLUE")
public class BlueFoundationParkWall extends BaseSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false);
        waitForStart();
        this.park(SAFE_WALL_DISTANCE_INCHES,-90);
    }
}
