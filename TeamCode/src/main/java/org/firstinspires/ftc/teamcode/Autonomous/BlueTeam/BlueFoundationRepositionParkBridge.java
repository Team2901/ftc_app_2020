package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

@Disabled
@Autonomous(name = "Blue Foundation Reposition Park Bridge", group = "_BLUE")
public class BlueFoundationRepositionParkBridge extends BlueSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null, true, 2);
        waitForStart();
        platformParkInner(Color.BLUE);
    }
}
