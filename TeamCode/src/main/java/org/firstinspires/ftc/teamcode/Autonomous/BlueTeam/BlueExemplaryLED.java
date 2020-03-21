package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BlueSkyStoneAuto;

import static org.firstinspires.ftc.teamcode.Hardware.ExemplaryBlinkinLED.LED_BLUE;

@Autonomous(name = "Blue Exemplary LED", group = "_BLUE")
public class BlueExemplaryLED extends BlueSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        init(false, false, null, false, LED_BLUE);
        waitForStart();
    }
}
