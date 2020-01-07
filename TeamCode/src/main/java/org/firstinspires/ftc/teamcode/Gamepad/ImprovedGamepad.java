package org.firstinspires.ftc.teamcode.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;

public class ImprovedGamepad {

    private final static float DEFAULT_STICK_DEAD_ZONE_VALUE = 0.1f;

    private final Gamepad hardwareGamepad;
    private final ElapsedTime timer;
    private final String name;
    private final float stickDeadZoneValue;

    public FloatButton left_stick_x;
    public FloatButton left_stick_y;
    public FloatButton right_stick_x;
    public FloatButton right_stick_y;
    public BooleanButton dpad_up;
    public BooleanButton dpad_down;
    public BooleanButton dpad_left;
    public BooleanButton dpad_right;
    public BooleanButton a;
    public BooleanButton b;
    public BooleanButton x;
    public BooleanButton y;
    public BooleanButton guide;
    public BooleanButton start;
    public BooleanButton back;
    public BooleanButton left_bumper;
    public BooleanButton right_bumper;
    public BooleanButton left_stick_button;
    public BooleanButton right_stick_button;
    public FloatButton left_trigger;
    public FloatButton right_trigger;

    public double raw_left_stick_radius = 0;
    public double raw_right_stick_radius = 0;

    public double left_stick_radius = 0;
    public double right_stick_radius = 0;

    // angle 0 = pushed forward, angle 90 = pushed left, etc
    public double left_stick_angle = 0;
    public double right_stick_angle = 0;

    public ImprovedGamepad(final Gamepad hardwareGamepad,
                           final ElapsedTime timer,
                           final String name) {
        this(hardwareGamepad, timer, name, DEFAULT_STICK_DEAD_ZONE_VALUE);
    }

    public ImprovedGamepad(final Gamepad hardwareGamepad,
                           final ElapsedTime timer,
                           final String name,
                           final float stickDeadZoneValue) {

        this.hardwareGamepad = hardwareGamepad;
        this.timer = timer;
        this.name = (null != name) ? name : "";
        this.stickDeadZoneValue = stickDeadZoneValue;

        this.left_stick_x = new FloatButton(this.name + "left_stick_x", stickDeadZoneValue, false);
        this.left_stick_y = new FloatButton(this.name + "left_stick_y", stickDeadZoneValue, true);
        this.right_stick_x = new FloatButton(this.name + "right_stick_x", stickDeadZoneValue, false);
        this.right_stick_y = new FloatButton(this.name + "right_stick_y", stickDeadZoneValue, true);

        this.dpad_up = new BooleanButton(this.name + "dpad_up");
        this.dpad_down = new BooleanButton(this.name + "dpad_down");
        this.dpad_left = new BooleanButton(this.name + "dpad_left");
        this.dpad_right = new BooleanButton(this.name + "dpad_right");
        this.a = new BooleanButton(this.name + "a");
        this.b = new BooleanButton(this.name + "b");
        this.x = new BooleanButton(this.name + "x");
        this.y = new BooleanButton(this.name + "y");
        this.guide = new BooleanButton(this.name + "guide");
        this.start = new BooleanButton(this.name + "start");
        this.back = new BooleanButton(this.name + "back");
        this.left_bumper = new BooleanButton(this.name + "left_bumper");
        this.right_bumper = new BooleanButton(this.name + "right_bumper");
        this.left_stick_button = new BooleanButton(this.name + "left_stick_button");
        this.right_stick_button = new BooleanButton(this.name + "right_stick_button");

        this.left_trigger = new FloatButton(this.name + "left_trigger", 0.25f);
        this.right_trigger = new FloatButton(this.name + "right_trigger", 0.25f);
    }

    public void update() {

        double time = timer.time();

        left_stick_x.update(hardwareGamepad.left_stick_x, time);
        left_stick_y.update(hardwareGamepad.left_stick_y, time);
        right_stick_x.update(hardwareGamepad.right_stick_x, time);
        right_stick_y.update(hardwareGamepad.right_stick_y, time);
        dpad_up.update(hardwareGamepad.dpad_up, time);
        dpad_down.update(hardwareGamepad.dpad_down, time);
        dpad_left.update(hardwareGamepad.dpad_left, time);
        dpad_right.update(hardwareGamepad.dpad_right, time);
        a.update(hardwareGamepad.a, time);
        b.update(hardwareGamepad.b, time);
        x.update(hardwareGamepad.x, time);
        y.update(hardwareGamepad.y, time);
        guide.update(hardwareGamepad.guide, time);
        start.update(hardwareGamepad.start, time);
        back.update(hardwareGamepad.back, time);
        left_bumper.update(hardwareGamepad.left_bumper, time);
        right_bumper.update(hardwareGamepad.right_bumper, time);
        left_stick_button.update(hardwareGamepad.left_stick_button, time);
        right_stick_button.update(hardwareGamepad.right_stick_button, time);
        left_trigger.update(hardwareGamepad.left_trigger, time);
        right_trigger.update(hardwareGamepad.right_trigger, time);


        raw_right_stick_radius = AngleUtilities.getRadius(right_stick_x.getRawValue(), right_stick_y.getRawValue());

       if (raw_right_stick_radius > stickDeadZoneValue) {
           right_stick_radius = (raw_right_stick_radius - stickDeadZoneValue) / (1 - stickDeadZoneValue);
           right_stick_angle = getJoystickAngle(right_stick_x, right_stick_y);
       } else {
           right_stick_radius = 0;
           // else keep last known angle 
       }

        raw_left_stick_radius = AngleUtilities.getRadius(left_stick_x.getRawValue(), left_stick_y.getRawValue());

        if (raw_left_stick_radius > stickDeadZoneValue) {
            left_stick_radius = (raw_left_stick_radius - stickDeadZoneValue) / (1 - stickDeadZoneValue);
            left_stick_angle = getJoystickAngle(left_stick_x, left_stick_y);
        } else {
            left_stick_radius = 0;
            // else keep last known angle 
        }
    }

    private double getJoystickAngle(FloatButton stick_x, FloatButton stick_y) {
        double angleRad = Math.atan2(stick_y.getRawValue(), stick_x.getRawValue());
        double angleDegrees = AngleUtilities.radiansDegreesTranslation(angleRad);
        // offset by 90 degrees so that forward is angle 0
        return AngleUtilities.getNormalizedAngle(angleDegrees - 90);
    }
}
