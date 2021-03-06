package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
@Autonomous(name = "Potentiometer Tester", group = "__TEST")
public class PotentiometerTest extends OpMode {

    AnalogInput potentiometer;
    double maxOgVoltage;

    @Override
    public void init() {
        potentiometer = hardwareMap.analogInput.get("potentiometer");
        maxOgVoltage = potentiometer.getMaxVoltage();
    }

    @Override
    public void loop() {

        double angle = (potentiometer.getVoltage()/potentiometer.getMaxVoltage())*270;
        double badAngle = ((potentiometer.getVoltage()/maxOgVoltage)*270);
        telemetry.addData("get Voltage" , potentiometer.getVoltage());
        telemetry.addData("get Max Voltage" , potentiometer.getMaxVoltage());
        telemetry.addData("Angle" , angle);
        telemetry.addData("Angle with regards to the original max voltage" ,badAngle);
        telemetry.addData("Connection Info" , potentiometer.getConnectionInfo());
        telemetry.update();


    }
}
