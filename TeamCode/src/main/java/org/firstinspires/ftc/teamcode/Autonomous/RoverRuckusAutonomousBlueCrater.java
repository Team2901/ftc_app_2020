package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.BaseRoverRuckusAuto.StartCorner.BLUE_CRATER;

@Autonomous(name = "Crater: Full", group = "RoverRuckus")
public class RoverRuckusAutonomousBlueCrater extends BaseRoverRuckusAuto {

    public RoverRuckusAutonomousBlueCrater() {
        super(BLUE_CRATER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        runOpModeCraterCorner();

        while(opModeIsActive()) {
            idle();
        }
    }
}
