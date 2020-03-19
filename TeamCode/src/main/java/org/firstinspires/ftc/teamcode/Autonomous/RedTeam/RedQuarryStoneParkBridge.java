package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.RedSkyStoneAuto;

@Autonomous(name = "Red Quarry Stone Park Bridge (11pt)", group = "_RED")
public class RedQuarryStoneParkBridge extends RedSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        init(false, false, null);
        waitForStart();
        quarrySkyStoneParkBridge(true, false, true);
    }
}
