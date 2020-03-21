package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.ExemplaryBlinkinLED;

public class RedSkyStoneAuto extends BaseSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
    }
    @Override
    public void init(boolean initWebCam, boolean setLiftServos, String message, boolean driveWheels, int teamColor){
        super.init(initWebCam, setLiftServos, message, driveWheels, teamColor);
        robot.blinkinLED.color = 2;
        robot.blinkinLED.setTeamPattern(ExemplaryBlinkinLED.TeamColorPattern.SOLID);
    }
}
