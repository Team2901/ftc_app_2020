package org.firstinspires.ftc.teamcode.Autonomous.BlueTeam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.BaseSkyStoneAuto;




@Autonomous(name = "Blue Quarry SkyStone Park Bridge", group = "blue")
public class BlueQuarrySkyStoneParkBridge extends BaseSkyStoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        init(true);
        // Wait for start
        waitForStart();

        quarrySkyStoneParkBridge(false);
    }
}