package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.RedSkyStoneAuto;


@Autonomous(name = "Red Exemplary LED", group = "_RED")
public class RedExemplaryLED extends RedSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null);
        waitForStart();
    }

}
