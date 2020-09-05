package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.KickOffSampleHardware;

@Autonomous(name = "Kick-off Sample Autonomous")
public class KickOffSampleAutonomous extends LinearOpMode {
    public KickOffSampleHardware robot = new KickOffSampleHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
    }

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

    /**
     * Turns the robot by the given turnAngle degrees
     *
     * @param turnAngle degrees to turn the robot by
     */
    public void turnByAngle(double turnAngle) {

        // Get robot's start angle
        double startAngle = robot.getAngle();

        // Calculate the robot's end angle (Normalize to be between -180 and 180)
        double targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);

        // Determine which direction the robot needs to turn (counter-clockwise or clockwise)
        int turnDirection;
        if (turnAngle >= 0) {
            // Turn counter-clockwise
            turnDirection = 1;
        } else {
            // Turn clockwise
            turnDirection = -1;
        }

        // Set the wheels to turn the correct direction
        robot.leftDrive.setPower(-0.25 * turnDirection);
        robot.rightDrive.setPower(0.25 * turnDirection);

        double currentAngle = robot.getAngle();

        if (turnDirection == 1) {
            // If turning counter-clockwise: update currentAngle until it is at or greater than the targetAngle
            while (AngleUnit.normalizeDegrees(currentAngle - targetAngle) < 0 && opModeIsActive()) {

                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = robot.getAngle();
            }
        } else {
            //  If turning clockwise: update currentAngle until it is at or less than the targetAngle
            while (AngleUnit.normalizeDegrees(currentAngle - targetAngle) > 0 && opModeIsActive()) {

                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = robot.getAngle();
            }
        }

        // Stop the motors once we hit the targetAngle
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
