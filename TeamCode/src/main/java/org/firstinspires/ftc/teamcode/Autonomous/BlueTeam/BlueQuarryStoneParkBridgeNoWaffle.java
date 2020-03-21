package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

@Autonomous(name = "Blue Quarry Stone Park Bridge No Waffle (7pt)", group = "_BLUE")
public class BlueQuarryStoneParkBridgeNoWaffle extends BlueSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        init(false, false, null);
        waitForStart();
        quarrySkyStoneParkBridge(false, false, false);
    }
}
