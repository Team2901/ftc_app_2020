package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.KickOffSampleHardware;

@Autonomous(name = "Kick-off Sample Autonomous")
public class KickOffSampleAutonomous extends LinearOpMode {
    public KickOffSampleHardware robot = new KickOffSampleHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot's hardware
        robot.init(this.hardwareMap);

        // Make the robot wait for you to click start
        waitForStart();

        // Move forwards 6 inches
        moveInches(6);

        robot.armServo.setPosition(.25);
        sleep(1000);
        robot.armServo.setPosition(.75);
    }

    /**
     * Moves robot forward given number of inches (or backwards if negative)
     *
     * @param inches number of inches to move by
     */
    public void moveInches(double inches) {
        // Calculate the target position
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        // Set the target position for each motor
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + ticks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + ticks);

        // Set the motors to run to a target position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motors (at half speed)
        robot.leftDrive.setPower(.5);
        robot.rightDrive.setPower(.5);

        // Display the motors current positions while they are moving
        // Note: Always check opModeIsActive() when using while loops
        while (opModeIsActive() &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            telemetry.addData("Current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.rightDrive.getCurrentPosition());

            telemetry.update();
        }

        // Stop the motors
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Reset the motors to run using encoders
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
