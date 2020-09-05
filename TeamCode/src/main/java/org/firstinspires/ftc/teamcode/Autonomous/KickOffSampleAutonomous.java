package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.KickOffSampleHardware;

@Autonomous (name = "Kick-off Sample Autonomous")
public class KickOffSampleAutonomous extends LinearOpMode {
    public KickOffSampleHardware robot = new KickOffSampleHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
    }
    public void moveInches (double inches){
        //calculating target position
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        //setting our target position
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + ticks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + ticks);

        //setting our motors to run to our target position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //starting motors
        robot.leftDrive.setPower(.5);
        robot.rightDrive.setPower(.5);

        while(opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){
            telemetry.addData("Current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.rightDrive.getCurrentPosition());

            telemetry.update();
        }

        //stop motors
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //resetting motors to run using encoders
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
