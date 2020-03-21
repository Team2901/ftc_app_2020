package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

@Autonomous(name = "Blue Quarry SkyStone Park Bridge (14pt)", group = "_BLUE")
public class BlueQuarrySkyStoneParkBridgeNoWaffle extends BlueSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(true, false, null, true, 2);
        waitForStart();
        quarrySkyStoneParkBridge(false, true, false);
    }
}