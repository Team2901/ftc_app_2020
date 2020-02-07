package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Disabled
@Autonomous(name = "Sky_Blue_Platform_Park_Inner", group = "BLUE")
public class PlatformParkInner extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        platformParkInner(Color.BLUE);
    }
}
