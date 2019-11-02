package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "SkyStoneLoadingAuto")

public class SkyStoneLoadingAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        1.) Find skystone
        2.) Get skystone
            Move forward, activate intake
        3.) Move right forward direction
        4.) Turn 90 degrees counterclockwise
            Find the waffle using the camera
        5.) If the waffle is not in the corner, put it in the corner
        6.) If the waffle is already in the corner
            Move to waffle, place block
         7.) Place Capstone on block
         */
        
        /*
        1.) Move waffle to the middle position
            Move forward, grab, drag back
        2.) Move right
            To go underneath the bridge
        3.) Find skystone
            Identify if there are three stones, if not, go to next location
        4.) Get skystone
        5.) Move right forward direction
        6.) Turn 180 degrees clockwise
        7.) Place block
        8.) Place Capstone on block
        9.) Return to starting position
            Go left, forward, and then left, turn 180 degrees
        10.) Go to step 3
         */

    }
}
