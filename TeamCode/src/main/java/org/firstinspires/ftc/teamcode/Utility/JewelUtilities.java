package org.firstinspires.ftc.teamcode.Utility;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcontroller.internal.JewelFinder;

public class JewelUtilities {

    public static Bitmap getBabyBitmap(Bitmap bitmap, JewelFinder jewel) {
        return BitmapUtilities.getBabyBitmap(bitmap, jewel.boxLeftXPct, jewel.boxTopYPct, jewel.boxRightXPct, jewel.boxBotYPct);
    }

}
