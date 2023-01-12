package org.firstinspires.ftc.teamcode.cameraCalibration;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionPipeline extends OpenCvPipeline
{
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();

    private final Scalar greenLower = new Scalar(45, 100, 50);
    private final Scalar greenUpper = new Scalar(75, 255, 255);

    private final Scalar yellowLower = new Scalar(20, 100, 50);
    private final Scalar yellowUpper = new Scalar(30, 255, 255);

    private final Scalar purpleLower = new Scalar(135, 100, 50);
    private final Scalar purpleUpper = new Scalar(165, 255, 255);

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

        // Return the input image
        return input;
    }
}