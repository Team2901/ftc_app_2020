package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

@Autonomous(name = "Blue Quarry SkyStone Park Bridge (19pt)", group = "_BLUE")
public class BlueQuarrySkyStoneParkBridge extends BlueSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(true, false, null);
        waitForStart();
        quarrySkyStoneParkBridge(false, true, true);
    }
}