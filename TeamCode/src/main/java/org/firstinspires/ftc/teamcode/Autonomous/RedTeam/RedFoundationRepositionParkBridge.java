package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.RedSkyStoneAuto;

@Disabled
@Autonomous(name = "Red Foundation Reposition Park Bridge", group = "_RED")
public class RedFoundationRepositionParkBridge extends RedSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null, true, 2);
        waitForStart();
        platformParkInner(Color.RED);
    }
}
