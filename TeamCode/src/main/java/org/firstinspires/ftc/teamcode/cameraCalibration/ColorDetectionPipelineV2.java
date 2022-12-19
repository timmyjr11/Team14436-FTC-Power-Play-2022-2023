package org.firstinspires.ftc.teamcode.cameraCalibration;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionPipelineV2 extends OpenCvPipeline
{
    Mat hsv = new Mat();
    Mat mask = new Mat();

    Scalar greenLower = new Scalar(45, 100, 50);
    Scalar greenUpper = new Scalar(75, 255, 255);

    Scalar yellowLower = new Scalar(20, 100, 50);
    Scalar yellowUpper = new Scalar(30, 255, 255);

    Scalar purpleLower = new Scalar(135, 100, 50);
    Scalar purpleUpper = new Scalar(165, 255, 255);

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert the input image to the HSV color space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create a mask for each of the colors we want to detect
        Core.inRange(hsv, greenLower, greenUpper, mask);
        Mat greenMask = mask.clone();

        Core.inRange(hsv, yellowLower, yellowUpper, mask);
        Mat yellowMask = mask.clone();

        Core.inRange(hsv, purpleLower, purpleUpper, mask);
        Mat purpleMask = mask.clone();

        // Set all pixels in the mask to white and all other pixels to black
        Core.bitwise_not(mask, mask);
        input.setTo(new Scalar(255, 255, 255), mask);

        // Count the number of non-zero pixels in each of the masks
        int greenCount = Core.countNonZero(greenMask);
        int yellowCount = Core.countNonZero(yellowMask);
        int purpleCount = Core.countNonZero(purpleMask);

        // Determine the dominant color in the image
        String dominantColor = "";
        if (greenCount > yellowCount && greenCount > purpleCount)
        {
            dominantColor = "Green";
        }
        else if (yellowCount > greenCount && yellowCount > purpleCount)
        {
            dominantColor = "Yellow";
        }
        else if (purpleCount > greenCount && purpleCount > yellowCount)
        {
            dominantColor = "Purple";
        }
        else
        {
            dominantColor = "None";
        }

        // Return the input image
        return input;
    }
}