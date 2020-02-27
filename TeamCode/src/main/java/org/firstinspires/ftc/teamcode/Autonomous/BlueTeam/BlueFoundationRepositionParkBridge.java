package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Disabled
@Autonomous(name = "Blue Foundation Reposition Park Bridge", group = "_BLUE")
public class BlueFoundationRepositionParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false);
        waitForStart();
        platformParkInner(Color.BLUE);
    }
}
