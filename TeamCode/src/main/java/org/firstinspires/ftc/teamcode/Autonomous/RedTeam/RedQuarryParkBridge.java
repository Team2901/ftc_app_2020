package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name = "Red Quarry Park Bridge (5pt)", group = "_RED")
public class RedQuarryParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null);
        waitForStart();
        this.park (PARK_BRIDGE_INCHES, -90);
    }
}
