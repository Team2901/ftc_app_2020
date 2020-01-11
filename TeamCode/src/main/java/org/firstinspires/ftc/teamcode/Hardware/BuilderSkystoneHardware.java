package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

public class BuilderSkystoneHardware extends BaseSkyStoneHardware{

    public final static double MOTOR_POWER_RATIO = .25;
    public final static double WHEEL_SERVO_GEAR_RATIO = 1.25;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan(WIDTH_OF_ROBOT/ LENGTH_OF_ROBOT);
    public final static int SERVO_MAX_ANGLE = 280;
    public final static double FRONT_LEFT_OFFSET = 0.04;
    public final static double FRONT_RIGHT_OFFSET = .33;
    public final static double BACK_LEFT_OFFSET = .14;
    public final static double BACK_RIGHT_OFFSET = 0.06;
    public final static double INCHES_TO_ENCODER = 22;
    // 103.6 ticks per rev *2motor revolutions per wheel revolution/3pi for wheel rev. to in
    public final static double WHEEL_MIN_ANGLE = 0;
    public final static double WHEEL_MAX_ANGLE = SERVO_MAX_ANGLE * WHEEL_SERVO_GEAR_RATIO;


    public static final double LEFT_GRABBER_MAX = 1.00;
    public static final double RIGHT_GRABBER_MIN = 0;

    public BuilderSkystoneHardware(){
        super (WIDTH_OF_ROBOT,
                LENGTH_OF_ROBOT,
                WHEEL_SERVO_GEAR_RATIO,
                SERVO_MAX_ANGLE,
                FRONT_LEFT_OFFSET,
                FRONT_RIGHT_OFFSET,
                BACK_LEFT_OFFSET,
                BACK_RIGHT_OFFSET,
                INCHES_TO_ENCODER);
    }


    public void init(HardwareMap hwMap) {

        super.init(hwMap);

        //Platform Grabber
        Servo leftGrabber = hwMap.get(Servo.class, "Left_grabber");
        Servo rightGrabber = hwMap.get(Servo.class, "Right_grabber");
        leftGrabber.setPosition(LEFT_GRABBER_MAX);
        rightGrabber.setPosition(RIGHT_GRABBER_MIN);

    }
}

