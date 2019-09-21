import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester")
public class ServoTester extends OpMode {
    Servo mrServo;

    @Override
    public void init() {
        mrServo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if (gamepad1.x){
            mrServo.setPosition(0);
        }
        if (gamepad1.a){
            mrServo.setPosition(1.0/3);
        }
        if (gamepad1.b){
            mrServo.setPosition(2.0/3);
        }
        if (gamepad1.y){
            mrServo.setPosition(1.0);

        }
        telemetry.addData("CurrentPosition",mrServo.getPosition());
        telemetry.addData("Press X","goes to position 0");
        telemetry.addData("Press A","goes to position 0.33");
        telemetry.addData("Press B","goes to position 0.66");
        telemetry.addData("Press Y","goes to position 1.0");
    }
}
