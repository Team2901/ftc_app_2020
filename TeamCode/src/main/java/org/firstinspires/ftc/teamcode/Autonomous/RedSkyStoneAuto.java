package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;
import org.firstinspires.ftc.teamcode.Hardware.ExemplaryBlinkinLED;

public class RedSkyStoneAuto extends BaseSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
    }
    @Override
    public void init(boolean initWebCam, boolean setLiftServos, String message){
        super.init(initWebCam, setLiftServos, message);
        robot.blinkinLED.color = 2;
        robot.blinkinLED.setTeamPattern(ExemplaryBlinkinLED.TeamColorPattern.SOLID);
    }
}
