package org.firstinspires.ftc.teamcode.NewProgrammers;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;

@Disabled
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
        SkytoneScanner(Color.RED);
    }
}
