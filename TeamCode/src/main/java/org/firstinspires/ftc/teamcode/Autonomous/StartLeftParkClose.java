package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

@Autonomous (name="StartLeftParkClose")
public class StartLeftParkClose extends LinearOpMode {
    final static double TICKS_PER_INCH = 2240/(3*Math.PI);

    public final SkystoneHardware robot = new SkystoneHardware();
    @Override
    public void waitForStart(){

    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.goToPosition(22 , this);
        robot.bridgeTickler.setPosition(1);


    }
}
