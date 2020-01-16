package org.firstinspires.ftc.teamcode.Autonomous.RedTeam;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Autonomous(name = "Sky_Red_Scanner", group = "_RED")
public class SkystoneScanner extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        String initVuforiaErrorMessage = robot.initWebCamera(hardwareMap);

        robot.webCamera.activateTfod();

        if (robot.webCamera.hasError()) {
            telemetry.addData("FAILED!",  robot.webCamera.errorMessage);

        } else {
            telemetry.addData("Successful!", "");
        }

        robot.swerveStraight(0, 0);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        SkystonsScanner(Color.RED);
    }
}
