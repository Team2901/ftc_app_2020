package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneHardware;
import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

import static org.firstinspires.ftc.teamcode.Utility.AngleUtilities.getNormalizedAngle;
@TeleOp(name = "One Controller SKYSTONE TELEOP ", group = "competition")
public class OneControllerSkystoneTeleOp extends OpMode {


    public static final double WHEEL_POWER_RATIO = .25;

    public final static double WHEEL_SERVO_GEAR_RATIO = .3;
    public final static double WIDTH_OF_ROBOT = 13.5;
    public final static double LENGTH_OF_ROBOT = 13.5;
    //This is the angle Phi that we defined in the math done before this
    public final static double TURN_ANGLE = Math.atan(WIDTH_OF_ROBOT / LENGTH_OF_ROBOT);
    public final static int SERVO_MAX_ANGLE = 2727;
    public final static int SERVO_MIN_ANGLE = 0;
    public final static double FRONT_LEFT_OFFSET = .11;
    public final static double FRONT_RIGHT_OFFSET = .13;
    public final static double BACK_LEFT_OFFSET = .1;
    public final static double BACK_RIGHT_OFFSET = .11;
    private boolean isYPressed;
    private boolean isXPressed;
    private boolean isBPressed;
    private boolean isAPressed;
    double power = 0;
    int step = 0;
    int topStep = 0;

    public final static double WHEEL_MAX_ANGLE = SERVO_MAX_ANGLE * WHEEL_SERVO_GEAR_RATIO;

    public class SwerveWheel {
        double targetAngle = 0;
        int modifier = 1;
        double offset = 0;

        public void setTargetAndModifier(double targetAngle, int modifier) {
            this.targetAngle = targetAngle;
            this.modifier = modifier;
        }

        public double wheelAngleToServoPosition(double wheelAngle) {
            double servoAngle = wheelAngleToServoAngle(wheelAngle);
            double servoPosition = servoAngleToServoPosition(servoAngle);

            return servoPosition;

        }

        public double wheelAngleToServoPosition() {

            return wheelAngleToServoPosition(targetAngle);

        }

        public double wheelAngleToServoAngle(double wheelAngle) {
            double servoAngle = wheelAngle / WHEEL_SERVO_GEAR_RATIO;
            return servoAngle;
        }

        public double servoAngleToServoPosition(double servoAngle) {
            double servoPosition = (servoAngle / SERVO_MAX_ANGLE) + offset;
            return servoPosition;
        }

        public SwerveWheel(double offset) {
            this.offset = offset;
        }
    }

    public class SwerveWheels {
        SwerveWheel frontLeftMotor = new SwerveWheel(FRONT_LEFT_OFFSET);
        SwerveWheel frontRightMotor = new SwerveWheel(FRONT_RIGHT_OFFSET);
        SwerveWheel backLeftMotor = new SwerveWheel(BACK_LEFT_OFFSET);
        SwerveWheel backRightMotor = new SwerveWheel(BACK_RIGHT_OFFSET);

        public void setTargetAndModifier(double targetAngle, int modifier) {
            frontLeftMotor.setTargetAndModifier(targetAngle, modifier);
            frontRightMotor.setTargetAndModifier(targetAngle, modifier);
            backLeftMotor.setTargetAndModifier(targetAngle, modifier);
            backRightMotor.setTargetAndModifier(targetAngle, modifier);
        }
    }

    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        //The y position is -1 to correct the joystick directions
        robot.swerveStraight(0, 0);
    }

    @Override
    public void loop() {
        double joystickPositionX = gamepad1.left_stick_x;
        double joystickPositionY = -gamepad1.left_stick_y;

        double radius = Math.sqrt(Math.pow(joystickPositionX, 2) + Math.pow(joystickPositionY, 2));

        telemetry.addData("X", joystickPositionX);
        telemetry.addData("Y", joystickPositionY);

        if (Math.abs(gamepad1.right_stick_x) > .1) {
            double power = getWheelPower(gamepad1.right_stick_x, 0, gamepad1.left_bumper);
            robot.swerveTurn(power);
        } else if (AngleUtilities.getRadius(joystickPositionX, joystickPositionY) > .2) {
            double power = getWheelPower(joystickPositionX, joystickPositionY, gamepad1.left_bumper);
            double joyWheelAngle = joystickPositionToWheelAngle(joystickPositionX, joystickPositionY);
            robot.swerveStraight(joyWheelAngle, power);
        } else {
            robot.setWheelMotorPower(0,0,0,0);
        }

        //makes lift go up by levels
        if (gamepad1.left_bumper) {
            liftStepper();
            // WRIST > .25 DON'T MOVE LIFT
            // CLAW >.25 DON'T MOVE LIFT
        }

        if(gamepad1.left_bumper) {
            //GOT THIS OVERLOADED DONT FORGET
            if (gamepad1.right_bumper) {
                robot.crane.setPosition(robot.crane.getPosition() + .005);
            } else if (gamepad1.left_bumper) {
                robot.crane.setPosition(robot.crane.getPosition() - .005);
            }

            if (gamepad1.x) {
                robot.wrist.setPosition(robot.wrist.getPosition() + .01);

            } else if (gamepad1.y) {
                robot.wrist.setPosition(robot.wrist.getPosition() - .01);
            }
            telemetry.addData("wrist Position:", robot.wrist.getPosition());

            if (robot.isOkayToOpen() == true) {

                if (gamepad1.a) {
                    robot.jaw.setPosition(robot.jaw.getPosition() + .01);
                } else if (gamepad1.b) {
                    robot.jaw.setPosition(robot.jaw.getPosition() - .01);
                }
            }
        }
        telemetry.addData("jaw Position: ", robot.jaw.getPosition());

        telemetry.addData("frontLeft", String.format("tarAngle: %.2f   mod:%d",
                robot.swerveWheels.frontLeftMotor.targetAngle, robot.swerveWheels.frontLeftMotor.modifier));
        telemetry.addData("frontRight", String.format("tarAngle: %.2f   mod:%d",
                robot.swerveWheels.frontRightMotor.targetAngle, robot.swerveWheels.frontRightMotor.modifier));
        telemetry.addData("backLeft", String.format("tarAngle: %.2f   mod:%d",
                robot.swerveWheels.backLeftMotor.targetAngle, robot.swerveWheels.backLeftMotor.modifier));
        telemetry.addData("backRight", String.format("tarAngle: %.2f   mod:%d",
                robot.swerveWheels.backRightMotor.targetAngle, robot.swerveWheels.backRightMotor.modifier));

        telemetry.update();
    }

    public double getWheelPower(double x, double y, boolean pause) {
        if (pause) {
            return 0;
        } else {
            return AngleUtilities.getRadius(x,y) * WHEEL_POWER_RATIO;
        }
    }

    /*This method finds our desired angle based on the joysticks. We want out robot's wheels to
    follow the position of our joystick, so we find the angle of our joysticks position like it is
    a position on the coordinate plane
     */
    public double joystickPositionToWheelAngle(double joystickPositionX, double joystickPositionY) {
        double wheelAngleRad = Math.atan2(joystickPositionY, joystickPositionX);
        double wheelAngle = radiansDegreesTranslation(wheelAngleRad) - 90;
        double wheelAngleStandarized = AngleUtilities.getPositiveNormalizedAngle(wheelAngle);
        return wheelAngleStandarized;
    }

    public double radiansDegreesTranslation(double radians) {
        double degrees = radians * 180 / Math.PI;
        return degrees;

    }

    public int liftStepper() {
        // WRIST > .25 DON'T MOVE LIFT
        // CLAW >.25 DON'T MOVE LIFT
        if(robot.wrist.getPosition()> .25)
        {
            if(robot.jaw.getPosition()> .25)
            {
                telemetry.addLine("I could self damage.");
            }
        }
        else{
            if (this.gamepad1.y && !isYPressed) {
                step = step + 1;
                if (step > 5) {
                    step = 5;
                }
                power = .5;
                //up by one
            }
            if (this.gamepad1.a && !isAPressed) {
                step = step - 1;
                if (step < 0) {
                    step = 0;
                }
                power = .5;
                //down by one
            }

            if (this.gamepad1.b && !isBPressed) {
                topStep = step;
                step = 0;
                power = .7;
                //to bottom
            }

            if (this.gamepad1.x && !isXPressed) {
                step = topStep;
                power = .7;
                //to top
            }

        }

        telemetry.addData("Step", step);
        telemetry.addData("Top Step", topStep);
        telemetry.update();

        isYPressed = gamepad1.y;
        isXPressed = gamepad1.x;
        isBPressed = gamepad1.b;
        isAPressed = gamepad1.a;


        //DcMotor lift = this.hardwareMap.dcMotor.get("lift");
        int targetPosition = step * 750;

        robot.lift.setTargetPosition(targetPosition);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(power);

        telemetry.update();
        return step;

    }

}


