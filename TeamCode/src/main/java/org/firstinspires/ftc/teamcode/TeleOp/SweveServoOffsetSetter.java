package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.BaseSkyStoneHardware;
import org.firstinspires.ftc.teamcode.Hardware.BuilderSkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.FileUtilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Servo Swerve Offset Setter", group = "TEST")
public class SweveServoOffsetSetter extends OpMode {

    String CONFIG_FILENAME = "servo_offset_config.txt";

    BuilderSkystoneHardware robot = new BuilderSkystoneHardware();
    ImprovedGamepad impGamepad;
    ElapsedTime timer = new ElapsedTime();

    Servo servoUnderTest;

    int servoIndex;

    @Override
    public void init() {
        impGamepad = new ImprovedGamepad(this.gamepad1, this.timer, "GP");

        robot.init(hardwareMap);

        robot.swerveStraight(0,0);
        
        telemetry.update();
    }

    @Override
    public void loop() {
        impGamepad.update();

        if(this.impGamepad.dpad_right.isInitialPress()){
            servoIndex++;
            if(servoIndex >= robot.swerveWheels.length){
                servoIndex = 0;
            }
        } else if(this.impGamepad.dpad_left.isInitialPress()) {
            servoIndex--;
            if(servoIndex < 0){
                servoIndex = robot.swerveWheels.length - 1;
            }
        }

        servoUnderTest = robot.swerveWheels[servoIndex].servo;

        if(servoUnderTest != null){
            if(this.impGamepad.left_bumper.isInitialPress()){
                servoUnderTest.setPosition(servoUnderTest.getPosition()-0.1);
            } else if(this.impGamepad.right_bumper.isInitialPress()){
                servoUnderTest.setPosition(servoUnderTest.getPosition()+0.1);
            } else if(this.impGamepad.left_trigger.isInitialPress()){
                servoUnderTest.setPosition(servoUnderTest.getPosition()-0.01);
            } else if(this.impGamepad.right_trigger.isInitialPress()) {
                servoUnderTest.setPosition(servoUnderTest.getPosition()+0.01);
            }

            robot.offsets.set(servoIndex, servoUnderTest.getPosition());

            if (this.impGamepad.a.isInitialPress()) {
                robot.swerveWheels[servoIndex].setOffset(servoUnderTest.getPosition());
            }

            telemetry.addData("Left bumper","-0.1");
            telemetry.addData("Right bumper","+0.1");
            telemetry.addData("Left trigger","-0.01");
            telemetry.addData("Right trigger","+0.01");
            telemetry.addData("Servo name",robot.swerveWheels[servoIndex].name);
            telemetry.addData("Position", servoUnderTest.getPosition());
            telemetry.addData("Offset", robot.swerveWheels[servoIndex].offset);
            telemetry.addData("Rel Pos", servoUnderTest.getPosition() - robot.swerveWheels[servoIndex].offset);
            telemetry.addData("hardMin", robot.swerveWheels[servoIndex].hardMinWheelAngle);
            telemetry.addData("hardMax", robot.swerveWheels[servoIndex].hardMaxWheelAngle);
        }

        telemetry.addData("D Pad Right/Left", "Increment/decrement servos");

        telemetry.update();

    }

    @Override
    public void stop() {

        super.stop();

        String errorMsg = robot.writeOffsets();
        if (errorMsg != null) {
            throw new RuntimeException(errorMsg);
        }
    }
}
