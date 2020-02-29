package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name = "Blue Quarry SkyStone Park Bridge (14pt)", group = "_BLUE")
public class BlueQuarrySkyStoneParkBridgeNoWaffle extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(true, false, null);
        waitForStart();
        quarrySkyStoneParkBridge(false, true, false);
    }
}