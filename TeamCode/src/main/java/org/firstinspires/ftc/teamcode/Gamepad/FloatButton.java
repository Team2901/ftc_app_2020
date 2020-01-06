package org.firstinspires.ftc.teamcode.Gamepad;


public class FloatButton extends Button<Float> {

    private final static float DEFAULT_DEAD_ZONE_VALUE = 0.0f;
    private final static float DEFAULT_RAW_VALUE = 0.0f;

    private boolean invertValue;
    protected float deadZoneValue;

    public FloatButton(final String name) {
        this(name, DEFAULT_DEAD_ZONE_VALUE, false);
    }

    public FloatButton(final String name,
                       final float deadZoneValue) {
        this(name, deadZoneValue, false);
    }

    public FloatButton(final String name,
                       final float deadZoneValue,
                       final boolean invertValue) {
        super(name);
        this.deadZoneValue = deadZoneValue;
        this.invertValue = invertValue;
        update(DEFAULT_RAW_VALUE, 0.0);
    }

    @Override
    protected boolean isPressed(final Float rawValue) {
        return (null != rawValue) && (Math.abs(rawValue) > deadZoneValue);
    }

    @Override
    public Float getRawValue() {
        if (null != rawValue && rawValue != 0) {
            return this.invertValue ? -rawValue : rawValue;
        } else {
            return 0f;
        }
    }

    @Override
    public Float getValue() {

        if (this.isPressed()) {
            final Float rawValue = this.getRawValue();
            return (rawValue - (Math.signum(rawValue) * deadZoneValue)) / (1 - deadZoneValue);
        } else {
            return 0.0f;
        }
    }
}