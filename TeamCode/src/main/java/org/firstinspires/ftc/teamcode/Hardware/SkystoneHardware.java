package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.ToolBox;
import org.firstinspires.ftc.teamcode.Autonomous.ToolBox;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

public class SkystoneHardware extends BaseSkyStoneHardware {

    public final static double WHEEL_SERVO_GEAR_RATIO = .3;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan(WIDTH_OF_ROBOT/ LENGTH_OF_ROBOT);
    public final static int SERVO_MAX_ANGLE = 2727;
    public final static double FRONT_LEFT_OFFSET = .0;
    public final static double FRONT_RIGHT_OFFSET = .0;
    public final static double BACK_LEFT_OFFSET = .0;
    public final static double BACK_RIGHT_OFFSET = .0;

    public final static double WHEEL_MIN_ANGLE = 0;
    public final static double WHEEL_MAX_ANGLE =  SERVO_MAX_ANGLE*WHEEL_SERVO_GEAR_RATIO;

    private final ToolBox toolBox;

    public SkystoneHardware(){

        super(WIDTH_OF_ROBOT,
                LENGTH_OF_ROBOT,
                WHEEL_SERVO_GEAR_RATIO,
                SERVO_MAX_ANGLE,
                FRONT_LEFT_OFFSET,
                FRONT_RIGHT_OFFSET,
                BACK_LEFT_OFFSET,
                BACK_RIGHT_OFFSET,
                0
              );

        this.toolBox = new ToolBox(this);

    }

    public void init(HardwareMap hwMap) {
        super.init(hwMap);
    }

    public void platformParkInner(){
        //Step one: turn wheels 90 degrees counterclockwise and go forward 1 ft.
        moveStraight(0.5,1);
        //Step two: turn wheels 90 degrees clockwise and go forward 1 ft.
        moveStraight(0.5,1);
        //Step three: turn wheels 90 degrees counterclockwise and go forward 1.5 ft.
        moveStraight(0.5,2);
        //Step four: stop
    }

    public void setWheelTargetPositions (int position){
        frontLeft.setTargetPosition(position*swerveWheels.frontLeftMotor.modifier);
        frontRight.setTargetPosition(position*swerveWheels.frontRightMotor.modifier);
        backLeft.setTargetPosition(position*swerveWheels.backLeftMotor.modifier);
        backRight.setTargetPosition(position*swerveWheels.backRightMotor.modifier);
    }

}

