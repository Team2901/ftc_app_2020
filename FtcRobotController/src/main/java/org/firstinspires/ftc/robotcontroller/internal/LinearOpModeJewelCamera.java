package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;
import android.widget.FrameLayout;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public abstract class LinearOpModeJewelCamera extends LinearOpMode {

  public List<Integer> middleConfig;
  public List<Integer> leftConfig;
  public List<Integer> rightConfig;

  public JewelFinder jewelLeft;
  public JewelFinder jewelMiddle;
  public JewelFinder jewelRight;
}
