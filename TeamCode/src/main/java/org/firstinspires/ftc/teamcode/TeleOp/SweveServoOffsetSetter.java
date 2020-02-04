package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Gamepad.ImprovedGamepad;
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

    String[] servoNames = {"servoFrontLeft","servoFrontRight","servoBackLeft","servoBackRight"};

    int servoIndex;

    ArrayList<Servo> servoArrayList = new ArrayList<>();

    List<Double> offsets = new ArrayList<>();

    @Override
    public void init() {
        impGamepad = new ImprovedGamepad(this.gamepad1, this.timer, "GP");

        robot.init(hardwareMap);

        try {
            offsets = FileUtilities.readDoubleConfigFile(CONFIG_FILENAME);
        } catch (IOException e) {
        }

        if (offsets.size() < servoNames.length) {
            offsets = new ArrayList<>();
            for (int i = 0; i < servoNames.length; i++) {
                offsets.add(0.0);
            }
        }

        for(int i = 0; i < servoNames.length; i++){
            String servoName = servoNames[i];
            Servo servo = hardwareMap.servo.get(servoName);
            servoArrayList.add(servo);
            servo.setPosition(offsets.size() > i ? offsets.get(i) : 0.0);
            telemetry.addData("Servo"+i,servoArrayList.get(i));
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        impGamepad.update();

        if(this.impGamepad.dpad_right.isInitialPress()){
            servoIndex++;
            if(servoIndex >= servoArrayList.size()){
                servoIndex = 0;
            }
        } else if(this.impGamepad.dpad_left.isInitialPress()) {
            servoIndex--;
            if(servoIndex < 0){
                servoIndex = servoArrayList.size() - 1;
            }
        }

        servoUnderTest = servoArrayList.get(servoIndex);

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

            offsets.set(servoIndex, servoUnderTest.getPosition());

            telemetry.addData("Left bumper","-0.1");
            telemetry.addData("Right bumper","+0.1");
            telemetry.addData("Left trigger","-0.01");
            telemetry.addData("Right trigger","+0.01");
            telemetry.addData("Position", servoUnderTest.getPosition());
            telemetry.addData("Servo name",servoNames[servoIndex]);
        }

        telemetry.addData("D Pad Right/Left", "Increment/decrement servos");

        telemetry.update();

    }

    @Override
    public void stop() {

        super.stop();

        try {
            FileUtilities.writeConfigFile(CONFIG_FILENAME, offsets);
            telemetry.addData("Success writing to file", "");
        } catch (Exception e) {
            throw new RuntimeException("Error writing to file", e);
        }
    }
}
