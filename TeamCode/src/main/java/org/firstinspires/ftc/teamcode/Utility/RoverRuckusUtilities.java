package org.firstinspires.ftc.teamcode.Utility;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;
import java.util.List;


public class RoverRuckusUtilities {

    public static final String JEWEL_CONFIG_FILE_FORMAT = "jewelConfig%s.txt";
    public static final String JEWEL_BITMAP_FILE_FORMAT = "jewelBitmap%s.png";
    public static final String JEWEL_BITMAP_BW_FILE_FORMAT = "jewelBitmapBW%s.png";
    public static final String JEWEL_HUE_FILE_FORMAT = "jewelHues%s.txt";

    public static int[] getJewelHueCount(Bitmap bitmap,
                                         String name,
                                         LinearOpMode opMode) throws RuntimeException, InterruptedException {
        return getJewelHueCount(bitmap, name, opMode, true);
    }

    public static int[] getJewelHueCount(Bitmap bitmap,
                                         String name,
                                         LinearOpMode opMode,
                                         boolean writeFiles) throws RuntimeException, InterruptedException {
        try {
                final String configFilename = String.format(JEWEL_CONFIG_FILE_FORMAT, name);
                final List<Integer> config = FileUtilities.readIntegerConfigFile(configFilename);
                final Bitmap babyBitmap = BitmapUtilities.getBabyBitmap(bitmap, config);

                if (writeFiles) {
                    final Bitmap babyBitmapBW = ColorUtilities.blackWhiteColorDecider(babyBitmap, 25, 40, opMode);

                final String bitmapFilename = String.format(JEWEL_BITMAP_FILE_FORMAT, name);
                final String hueFilename = String.format(JEWEL_HUE_FILE_FORMAT, name);
                final String bwFileName = String.format(JEWEL_BITMAP_BW_FILE_FORMAT, name);

                FileUtilities.writeBitmapFile(bitmapFilename, babyBitmap);
                FileUtilities.writeHueFile(hueFilename, babyBitmap, opMode);
                FileUtilities.writeBitmapFile(bwFileName, babyBitmapBW);
            }

            return ColorUtilities.getColorCount(babyBitmap, 25, 55, opMode);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
