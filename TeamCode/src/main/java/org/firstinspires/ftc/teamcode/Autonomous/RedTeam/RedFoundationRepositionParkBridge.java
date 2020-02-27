package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Disabled
@Autonomous(name = "Red Foundation Reposition Park Bridge", group = "_RED")
public class RedFoundationRepositionParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        platformParkInner(Color.RED);
    }
}
