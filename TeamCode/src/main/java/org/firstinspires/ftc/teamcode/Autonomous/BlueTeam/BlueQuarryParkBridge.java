package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

@Autonomous(name = "Blue Quarry Park Bridge (5pt)", group = "_BLUE")
public class BlueQuarryParkBridge extends BlueSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null, true, 2);
        waitForStart();
        this.park (PARK_BRIDGE_INCHES, 90);
    }
}
