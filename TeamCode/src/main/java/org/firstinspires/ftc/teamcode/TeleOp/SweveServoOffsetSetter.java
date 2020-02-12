package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;

@TeleOp(name = "Servo Swerve Offset Setter", group = "TEST")
public class SweveServoOffsetSetter extends OpMode {

    BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    ImprovedGamepad improvedGamepad1;
    ElapsedTime timer = new ElapsedTime();

    int servoUnderTestIndex;

    @Override
    public void init() {
        improvedGamepad1 = new ImprovedGamepad(this.gamepad1, this.timer, "GP");

        robot.init(hardwareMap);

        robot.swerveStraight(0,0);
    }

    @Override
    public void loop() {

        robot.setWheelMotorPower(0, 0, 0, 0);

        boolean underTestChange = false;

        if (this.improvedGamepad1.dpad_right.isInitialPress()) {
            servoUnderTestIndex++;
            underTestChange = true;
            if (servoUnderTestIndex >= robot.allSwerveWheels.length) {
                servoUnderTestIndex = 0;
            }
        } else if (this.improvedGamepad1.dpad_left.isInitialPress()) {
            servoUnderTestIndex--;
            underTestChange = true;
            if (servoUnderTestIndex < 0) {
                servoUnderTestIndex = robot.allSwerveWheels.length - 1;
            }
        }

        final BaseSkyStoneHardware.SwerveWheel swerveWheelUnderTest = robot.allSwerveWheels[servoUnderTestIndex];

        if (underTestChange) {
            telemetry.speak(swerveWheelUnderTest.servoConfigName);
        } else {
            double targetPosition = swerveWheelUnderTest.servo.getPosition();

            if (this.improvedGamepad1.left_bumper.isInitialPress()) {
                targetPosition -= 0.1;
            } else if (this.improvedGamepad1.right_bumper.isInitialPress()) {
                targetPosition += 0.1;
            } else if (this.improvedGamepad1.left_trigger.isInitialPress()) {
                targetPosition -= 0.01;
            } else if (this.improvedGamepad1.right_trigger.isInitialPress()) {
                targetPosition += 0.01;
            }

            swerveWheelUnderTest.servo.setPosition(targetPosition);

            // Update offsets
            swerveWheelUnderTest.setOffset(targetPosition);
            robot.offsets.set(servoUnderTestIndex, targetPosition);
        }

        telemetry.addData("Servo name", swerveWheelUnderTest.servoConfigName);
        telemetry.addData("Position", swerveWheelUnderTest.servo.getPosition());
        telemetry.addData("Left/Right bumper", "-/+ 0.1");
        telemetry.addData("Left/Right trigger", "-/+ 0.01");
        telemetry.addData("A", "Update offset");
        telemetry.addData("D Pad Right/Left", "Increment/decrement servos");
    }

    @Override
    public void stop() {

        super.stop();

        String errorMessage = robot.writeOffsets();

        if (errorMessage != null) {
            throw new RuntimeException(errorMessage);
        }
    }
}
