package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created with Team 6183's Duckinator 3000
 */

<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/BaseDominatorXDrive.java
@Autonomous(name = "BaseDominatorXDrive", group = "DuckSquad")
public abstract class BaseDominatorXDrive extends LinearOpMode {
=======
public class BaseDominatorXDrive extends LinearOpMode {
>>>>>>> 4c5985b6ca3281bb65a259128457daec1a027139:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/BaseDominatorXDrive.java
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    //holonomic encoder counts are slightly innacurate and need to be tested due to different amounts of force and friction on the wheels depending on what you get
//please adjust personally to each program, we have accounted for slight slippage but just please make sure
    private int globalAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        waitForStart();
        if (opModeIsActive()){
            goForward(3071);
            rotate(-89);
            goForward(8531);
            rotate(90);
            goForward(1007);
            rotate(179);
            goForward(1988);
            rotate(-89);
            goForward(4203);

        }
    }
    public void motorReset() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void powerBusy() {
        fl.setPower(0.5);
        fr.setPower(0.5);
        bl.setPower(0.5);
        br.setPower(0.5);
        while ((fl.isBusy() && fr.isBusy())&&(bl.isBusy() && br.isBusy())){}
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void goForward(int gofront){
        motorReset();
        fl.setTargetPosition((int)Math.round(1.2*gofront));
        fr.setTargetPosition((int)Math.round(-1.2*gofront));
        bl.setTargetPosition((int)Math.round(1.2*gofront));
        br.setTargetPosition((int)Math.round(1.2*gofront ));
        powerBusy();
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    private void rotate(int degrees) {
        double flp, frp, blp, brp;
        resetAngle();
        if (degrees < 0) {   // turn right.
            flp = 0.5;
            frp = 0.5;
            blp = 0.5;
            brp = 0.5;
        }
        else if (degrees > 0) {   // turn left.
            flp = -0.5;
            frp = -0.5;
            blp = -0.5;
            brp = -0.5;
        }
        else return;
        fl.setPower(flp);
        fr.setPower(frp);
        bl.setPower(blp);
        br.setPower(brp);
        if (degrees < 0) {//right
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        } else {//left
            while (opModeIsActive() && getAngle() < degrees) {}
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        sleep(1000);
        resetAngle();
    }
}