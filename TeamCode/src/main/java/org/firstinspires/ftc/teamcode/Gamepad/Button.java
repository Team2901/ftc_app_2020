package org.firstinspires.ftc.teamcode.Gamepad;


public abstract class Button<T> {

    protected final String name;
    protected T rawValue;
    private Boolean pressed = null;

    private int pressedCounts = 0;
    private int releaseCounts = 0;

    private Double lastUpdateTime;
    private Double lastChangeTime = null;

    public Button(final String name) {
        this.name = name;
    }

    public void update(final T updateValue, final double updateTime) {

        final boolean newIsPressed = isPressed(updateValue);

        if (pressed != null && newIsPressed != pressed) {
            this.lastChangeTime = updateTime;
        }

        this.rawValue = updateValue;
        this.pressed = newIsPressed;
        this.lastUpdateTime = updateTime;

        if (isInitialValueChange()) {
            if (isPressed()) {
                this.pressedCounts++;
            } else {
                this.releaseCounts++;
            }
        }
    }

    protected abstract boolean isPressed(final T rawValue);

    public abstract T getValue();

    public String getName() {
        return name;
    }

    public T getRawValue() {
        return rawValue;
    }

    public Double getLastUpdateTime() {
        return lastUpdateTime;
    }

    public Double getLastChangeTime() {
        return lastChangeTime;
    }

    public boolean isPressed() {
        return pressed;
    }

    public boolean isReleased() {
        return !isPressed();
    }

    public double valueElapseTime() {
        return (null != lastChangeTime) ? lastUpdateTime - lastChangeTime : lastUpdateTime;
    }

    public boolean isInitialValueChange() {
        return lastUpdateTime.equals(lastChangeTime);
    }

    public boolean isInitialPress() {
        return isPressed() && isInitialValueChange();
    }

    public boolean isInitialRelease() {
        return isReleased() && isInitialValueChange();
    }

    public double getPressedElapseTime() {
        return isPressed() ? valueElapseTime() : 0;
    }

    public double getReleaseElapseTime() {
        return isReleased() ? valueElapseTime() : 0;
    }

    public int getPressedCounts() {
        return pressedCounts;
    }

    public int getReleaseCounts() {
        return releaseCounts;
    }

    @Override
    public String toString() {
        final StringBuilder sb = new StringBuilder("Button{");
        sb.append("name='").append(name).append('\'');
        sb.append(", rawValue=").append(rawValue);
        sb.append(", pressed=").append(pressed);
        sb.append('}');
        return sb.toString();
    }
}


