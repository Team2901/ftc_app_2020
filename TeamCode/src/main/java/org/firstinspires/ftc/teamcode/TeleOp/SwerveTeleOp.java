package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;

@TeleOp (name = "Swerve Teleop")
public class SwerveTeleOp extends OpMode {
    public final static double WHEEL_SERVO_GEAR_RATIO = 1/1;
    public final static double WIDTH_OF_ROBOT = 18;
    public final static double LENGTH_OF_ROBOT = 18;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan((.5*WIDTH_OF_ROBOT)/(.5*LENGTH_OF_ROBOT));
    public final static int SERVO_MAX_ANGLE = 190;
    Servo servoFrontLeft;


    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
       robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        if (gamepad1.right_stick_x>.1 || gamepad1.right_stick_x< -.1 ){
            swerveTurn(gamepad1.right_stick_x);

        }else{
            setDriveServoPosition(joystickPositionX,joystickPositionY);
            setPower(joystickPositionX,joystickPositionY , 1);
        }

    }

    public double wheelAngleToServoAngle(double wheelAngle){
        double servoAngle = WHEEL_SERVO_GEAR_RATIO * wheelAngle;
        return servoAngle;
    }

    public double servoAngleToServoPosition(double servoAngle){
       double servoPosition = servoAngle/SERVO_MAX_ANGLE;
       return servoPosition;
    }

    public double joystickPositionToWheelAngle (double joystickPositionX, double joystickPositionY){
        double wheelAngleRad = Math.atan2(joystickPositionY,joystickPositionX);
        double wheelAngle = radiansDegreesTranslation(wheelAngleRad);
        double wheelAngleStandarized = standardizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }

    public double radiansDegreesTranslation (double radians){
        double degrees = radians * 180 / Math.PI;
        return degrees;

    }
    //Converting -180 to 180 to 0 to 360
    public double standardizedAngle(double angle) {
        return (angle + 360) % 360;
    }

    public double getPower (double x, double y){
        double power = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
        return power;
    }

    public void setPower (double joystickPositionX, double joystickPositionY, double modifier){

        double power = modifier*getPower( joystickPositionX,  joystickPositionY);

        robot.backRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.frontLeft.setPower(power);

    }

    public void setDriveServoPosition (double joystickPositionX, double joystickPositionY){

        double servoPosition = joystickToServoPosition(joystickPositionX,joystickPositionY);


        telemetry.addData("servoPosition",servoPosition);
        telemetry.addData("power" , getPower(joystickPositionX,joystickPositionY));
        telemetry.update();

        setAllServos(servoPosition, servoPosition,servoPosition,servoPosition);

    }

    public void swerveTurn(double joyStickRightPosX) {

            double fRPos = 90 + TURN_ANGLE;
            double fLPos = 90 - TURN_ANGLE;
            double bLPos = 270 + TURN_ANGLE;
            double bRPos = 270 - TURN_ANGLE;

            setAllServos(wheelAngleToServoPosition(fRPos), wheelAngleToServoPosition(fLPos),
                    wheelAngleToServoPosition(bLPos), wheelAngleToServoPosition(bRPos));

            setPower(joyStickRightPosX, 0, Math.signum(joyStickRightPosX));

        }

    public double joystickToServoPosition(double joystickPositionX, double joystickPositionY){
        double wheelAngle = joystickPositionToWheelAngle(joystickPositionX,joystickPositionY);
        double servoAngle = wheelAngleToServoAngle(wheelAngle);
        double servoPosition = servoAngleToServoPosition(servoAngle);
        return servoPosition;

    }

    public double wheelAngleToServoPosition(double angle){
        double servoAngle = wheelAngleToServoAngle(angle);
        double servoPosition = servoAngleToServoPosition(servoAngle);
        return servoPosition;
    }

    public void setAllServos(double fRPos, double fLPos, double bLPos, double bRPos ){
        robot.servoFrontRight.setPosition(fRPos);
        robot.servoBackRight.setPosition(bRPos);
        robot.servoFrontLeft.setPosition(fLPos);
        robot.servoBackLeft.setPosition(bLPos);
    }
}
