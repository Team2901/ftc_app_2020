package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name = "Red Quarry SkyStone Park Bridge", group = "_RED")
public class RedQuarrySkyStoneParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(true);
        waitForStart();
        quarrySkyStoneParkBridge(true);
    }
}