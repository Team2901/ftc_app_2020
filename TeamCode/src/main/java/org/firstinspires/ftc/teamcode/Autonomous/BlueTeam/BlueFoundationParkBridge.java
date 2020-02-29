package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name = "Blue Foundation Park Bridge (5pt)", group = "_BLUE")
public class BlueFoundationParkBridge extends BaseSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null);
        waitForStart();
        this.park (PARK_BRIDGE_INCHES, -90);
    }
}
