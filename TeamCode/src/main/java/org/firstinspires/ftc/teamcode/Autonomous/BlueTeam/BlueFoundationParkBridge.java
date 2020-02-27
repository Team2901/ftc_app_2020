package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;


@Autonomous(name = "Blue Foundation Park Bridge", group = "_BLUE")
public class BlueFoundationParkBridge extends BaseSkyStoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.swerveStraight(0, 0);
        waitForStart();
        this.park (PARK_BRIDGE_INCHES, -90);
    }

}
