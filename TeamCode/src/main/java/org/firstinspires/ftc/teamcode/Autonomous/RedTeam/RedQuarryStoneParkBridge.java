package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name = "Red Quarry Stone Park Bridge (11pt)", group = "_RED")
public class RedQuarryStoneParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        init(false, false, null);
        waitForStart();
        quarrySkyStoneParkBridge(true, false);
    }
}
