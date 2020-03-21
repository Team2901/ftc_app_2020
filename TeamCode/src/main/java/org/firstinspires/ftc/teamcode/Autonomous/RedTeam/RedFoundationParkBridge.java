package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.RedSkyStoneAuto;


@Autonomous(name = "Red Foundation Park Bridge (5pt)", group = "_RED")
public class RedFoundationParkBridge extends RedSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null, true, 2);
        waitForStart();
        this.park (PARK_BRIDGE_INCHES, 90);
    }

}
