package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.RedSkyStoneAuto;

@Autonomous(name = "Red Quarry SkyStone Park Bridge No Waffle (15pt)", group = "_RED")
public class RedQuarrySkyStoneParkBridgeNoWaffle extends RedSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(true, false, null, true, 2);
        waitForStart();
        quarrySkyStoneParkBridge(true, true, false);
    }
}