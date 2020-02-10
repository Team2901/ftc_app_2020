package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingSkystoneHardware extends BaseSkyStoneHardware {

    public final static double WHEEL_SERVO_GEAR_RATIO = .3;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    public final static int SERVO_MAX_ANGLE = 2727;
    public final static double FRONT_LEFT_OFFSET = .0;
    public final static double FRONT_RIGHT_OFFSET = .0;
    public final static double BACK_LEFT_OFFSET = .0;
    public final static double BACK_RIGHT_OFFSET = .0;
    public final static double WHEEL_POWER_RATIO = 0.5;

    public ProgrammingSkystoneHardware(){

        super(WIDTH_OF_ROBOT,
                LENGTH_OF_ROBOT,
                WHEEL_SERVO_GEAR_RATIO,
                SERVO_MAX_ANGLE,
                FRONT_LEFT_OFFSET,
                FRONT_RIGHT_OFFSET,
                BACK_LEFT_OFFSET,
                BACK_RIGHT_OFFSET,
                0,
                WHEEL_POWER_RATIO);
    }

    public void init(HardwareMap hwMap) {
        super.init(hwMap);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}

