package org.firstinspires.ftc.robotcontroller.internal;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.Canvas;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by butterss21317 on 11/14/2017.
 */
@SuppressLint("DefaultLocale")
public class JewelFinder extends TextView implements View.OnTouchListener {
    public static final String JEWEL_CONFIG_FILE_FORMAT = "jewelConfig%s.txt";
    public static final String JEWEL_BITMAP_FILE_FORMAT = "jewelBitmap%s.png";
    public static final String JEWEL_BITMAP_BW_FILE_FORMAT = "jewelBitmapBW%s.png";
    public static final String JEWEL_HUE_FILE_FORMAT = "jewelHues%s.txt";

    private int pWidth;
    private int pHeight;
    private String name;
    private float dx, dy;

    // Pct is = to box percentage location.
    public int boxLeftXPct = 0;
    public int boxRightXPct = 20;
    public int boxTopYPct = 0;
    public int boxBotYPct = 20;

    public JewelFinder(Context context, int color, String name) {
        this(context, null, color, name);
    }

    public JewelFinder(Context context, AttributeSet attrs, int color, String name) {
        this(context, attrs, 0, color, name);
    }

    public JewelFinder(Context context, AttributeSet attrs, int defStyle, int color, String name) {
        super(context, attrs, defStyle);
        this.setBackgroundColor(color);
        this.setLayoutParams(new FrameLayout.LayoutParams(100, 100));
        this.setOnTouchListener(this);
        this.name = name;
        this.setText(name != null && !name.isEmpty() ? name.substring(0, 1) : "Unknown");
    }

    public void moveTo(List<Integer> config, int parentWidth, int parentHeight) {
        setBoxPct(config);
        setX(boxLeftXPct / 100.0f * parentWidth);
        setY(boxTopYPct / 100.0f * parentHeight);
        setLayoutParams(new FrameLayout.LayoutParams(100, 100));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // TODO: consider storing these as member variables to reduce
        // allocations per draw cycle.
        int paddingLeft = getPaddingLeft();
        int paddingTop = getPaddingTop();
        int paddingRight = getPaddingRight();
        int paddingBottom = getPaddingBottom();

        int contentWidth = getWidth() - paddingLeft - paddingRight;
        int contentHeight = getHeight() - paddingTop - paddingBottom;
        pWidth = ((View) this.getParent()).getWidth();
        pHeight = ((View) this.getParent()).getHeight();
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        Log.e("", "on  touch");

        float rawX = event.getRawX();
        float rawY = event.getRawY();

        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            dx = v.getX() - event.getRawX();
            dy = v.getY() - event.getRawY();
            return true;
        } else if (event.getAction() == MotionEvent.ACTION_MOVE) {
            float x = getXBound(rawX + dx);
            float y = getYBound(rawY + dy);

            boxLeftXPct = (int) ((x / pWidth) * 100);
            boxRightXPct = (int) (((x + this.getWidth()) / pWidth) * 100);
            boxTopYPct = (int) ((y / pHeight) * 100);
            boxBotYPct = (int) (((y + this.getHeight()) / pHeight) * 100);

            v.animate()
                    .x(x)
                    .y(y)
                    .setDuration(0)
                    .start();
            return true;
        }
        return false;
    }

    private float getXBound(float valueX) {
        if (valueX > pWidth - this.getWidth()) {// box x size
            return pWidth - this.getWidth();
        } else if (valueX < 0) {
            return 0;
        } else {
            return valueX;
        }
    }

    private float getYBound(float valueY) {
        if (valueY > pHeight - this.getHeight()) {//half box y size
            return pHeight - this.getHeight();
        } else if (valueY < 0) {
            return 0;
        } else {
            return valueY;
        }
    }

    public String getName() {
        return name;
    }

    public void setBoxPct(List<Integer> config) {
        boxLeftXPct = config.get(0);
        boxTopYPct = config.get(1);
        boxRightXPct = config.get(2);
        boxBotYPct = config.get(3);
    }

    public List<Integer> getBoxPct() {
        List<Integer> configValues = new ArrayList<>();
        configValues.add(boxLeftXPct);
        configValues.add(boxTopYPct);
        configValues.add(boxRightXPct);
        configValues.add(boxBotYPct);
        return configValues;
    }

    public String getConfigFileName() {
        return  String.format(JEWEL_CONFIG_FILE_FORMAT, getName());
    }

    public String getHueFileName() {
        return  String.format(JEWEL_HUE_FILE_FORMAT, getName());
    }

    public String getBitmapFileName() {
        return  String.format(JEWEL_BITMAP_FILE_FORMAT, getName());
    }
}