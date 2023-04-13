package org.firstinspires.ftc.teamcode.cameraCalibration;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class CompVisionForAuto extends OpenCvPipeline {
    Mat hsv = new Mat();
    Mat mask = new Mat();

    private static ConfigPos.colors color = ConfigPos.colors.None;

    public static int rectX = 0;
    public static int rectY = 0;
    public static int rectHeight = 0;
    public static int rectWidth = 0;

    public static double lowerGreenH = 50;
    public static double lowerGreenS = 50;
    public static double lowerGreenV = 25;

    public static double lowerYellowH = 20;
    public static double lowerYellowS = 200;
    public static double lowerYellowV = 50;

    public static double lowerPurpleH = 120;
    public static double lowerPurpleS = 0;
    public static double lowerPurpleV = 0;

    public static double upperGreenH = 90;
    public static double upperGreenS = 255;
    public static double upperGreenV = 255;

    public static double upperYellowH = 35;
    public static double upperYellowS = 255;
    public static double upperYellowV = 255;

    public static double upperPurpleH = 170;
    public static double upperPurpleS = 255;
    public static double upperPurpleV = 255;

    Scalar greenLower = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
    Scalar greenUpper = new Scalar(upperGreenH, upperGreenS, upperGreenV);

    Scalar yellowLower = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
    Scalar yellowUpper = new Scalar(upperYellowH, upperYellowS, upperYellowV);

    Scalar purpleLower = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
    Scalar purpleUpper = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input image to the HSV color space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        greenLower = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
        greenUpper = new Scalar(upperGreenH, upperGreenS, upperGreenV);

        yellowLower = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
        yellowUpper = new Scalar(upperYellowH, upperYellowS, upperYellowV);

        purpleLower = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
        purpleUpper = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

        // Create a mask for each of the colors we want to detect
        Core.inRange(hsv, greenLower, greenUpper, mask);
        Mat greenMask = mask.clone();

        Core.inRange(hsv, yellowLower, yellowUpper, mask);
        Mat yellowMask = mask.clone();

        Core.inRange(hsv, purpleLower, purpleUpper, mask);
        Mat purpleMask = mask.clone();

        // Set all pixels in the mask to white and all other pixels to black
        Core.bitwise_or(greenMask, yellowMask, mask);
        Core.bitwise_or(purpleMask, mask, mask);
        Core.bitwise_not(mask, mask);
        input.setTo(new Scalar(0, 0, 0), mask);

        // Count the number of non-zero pixels in each of the masks
        int greenCount = Core.countNonZero(greenMask);
        int yellowCount = Core.countNonZero(yellowMask);
        int purpleCount = Core.countNonZero(purpleMask);


        if (greenCount > yellowCount && greenCount > purpleCount) {
            color = ConfigPos.colors.green;
        }
        else if (yellowCount > greenCount && yellowCount > purpleCount) {
            color = ConfigPos.colors.yellow;
        }
        else if (purpleCount > greenCount && purpleCount > yellowCount) {
            color = ConfigPos.colors.purple;
        }
        else {
            color = ConfigPos.colors.None;
        }

        // Return the input image
        mask.release();
        hsv.release();
        yellowMask.release();
        purpleMask.release();
        greenMask.release();
        return input;
    }

    public ConfigPos.colors getColor() {
        return color;
    }
}