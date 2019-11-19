package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;


@Autonomous(name = "StartRightParkClose")
public class StartRightParkClose extends LinearOpMode {

   final static double TICKS_PER_INCH = 2240/(3*Math.PI);

    public final SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.goToPosition(22 , this);
        robot.bridgeTickler.setPosition(1);

        //turns right
       //if need to move (turn wheels to 0) pause then put down planeBreaker


    }





    //2240/3pi ticks per inch
    //wheels to the right
    //go foward and put down
    // or turn wheels right then go foward put down


}
