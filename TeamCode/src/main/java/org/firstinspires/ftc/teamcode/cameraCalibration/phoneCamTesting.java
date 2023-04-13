package org.firstinspires.ftc.teamcode.cameraCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pastCodes.ConfigPos;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Config
@Autonomous
public class phoneCamTesting extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    static ConfigPos.colors color = ConfigPos.colors.None;
    static ConfigPos.colors previousColor = ConfigPos.colors.None;

    OpenCvCamera phoneCam;

    int coneCounter = 0;

    ConfigPos.side side = ConfigPos.side.tbd;

    @SuppressWarnings("deprecation")
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera2(OpenCvInternalCamera2
                        .CameraDirection.BACK, cameraMonitorViewId);


        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        phoneCam.setPipeline(new CompVisionForAuto());

        telemetry.addLine("Opening Cameras...");
        telemetry.update();


        while (!isStarted()) {
            telemetry.addData("Color" , color);
            telemetry.addData("Previous color", previousColor);

            telemetry.addData("Side", side);
            telemetry.addData("Cone count", coneCounter);

            if (color != previousColor) {
                switch (color) {
                    case green:
                        previousColor = ConfigPos.colors.green;
                        break;
                    case purple:
                        previousColor = ConfigPos.colors.purple;
                        break;
                    case yellow:
                        previousColor = ConfigPos.colors.yellow;
                        break;
                }
                telemetry.update();
            }

            if (isStopRequested()) return;
            if (isStarted()) break;
        }

        // Make a variable that changes the lift height for each loop
    }

    @Config
    static class CompVisionForAuto extends OpenCvPipeline {

        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat box = new Mat();

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

        Rect rect = new Rect(rectX, rectY, rectWidth, rectHeight);
        Scalar rectangleColor = new Scalar(0, 0, 255);


        @Override
        public Mat processFrame(Mat input) {
            // Convert the input image to the HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Imgproc.rectangle(hsv, rect, rectangleColor, 2);

            box = hsv.submat(rect);

            // Create a mask for each of the colors we want to detect
            Core.inRange(box, greenLower, greenUpper, hsv);
            Mat greenMask = hsv.clone();

            Core.inRange(box, yellowLower, yellowUpper, hsv);
            Mat yellowMask = hsv.clone();

            Core.inRange(box, purpleLower, purpleUpper, hsv);
            Mat purpleMask = hsv.clone();

            // Set all pixels in the mask to white and all other pixels to black
            Core.bitwise_or(greenMask, yellowMask, hsv);
            Core.bitwise_or(purpleMask, hsv, mask);
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
            box.release();
            yellowMask.release();
            purpleMask.release();
            greenMask.release();
            return input;
        }
    }
}