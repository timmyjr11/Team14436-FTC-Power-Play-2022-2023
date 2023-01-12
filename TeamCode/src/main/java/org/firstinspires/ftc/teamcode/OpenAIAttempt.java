package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class OpenAIAttempt extends LinearOpMode {

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
        phoneCam.setPipeline(new ColorFilterPipeline());

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

    static class ColorFilterPipeline extends OpenCvPipeline {
        private Mat hsvMat = new Mat();
        private Mat mask = new Mat();
        private Rect rect;
        private Scalar lowerYellow = new Scalar(20, 100, 100);
        private Scalar upperYellow = new Scalar(30, 255, 255);
        private Scalar lowerGreen = new Scalar(50, 100, 100);
        private Scalar upperGreen = new Scalar(70, 255, 255);
        private Scalar lowerPurple = new Scalar(130, 100, 100);
        private Scalar upperPurple = new Scalar(145, 255, 255);
        private Scalar black = new Scalar(0, 0, 0);

        public ColorFilterPipeline() {
            rect = new Rect(new Point(150, 150), new Size(150, 150));
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV);

            Core.inRange(hsvMat, lowerYellow, upperYellow, mask);
            input.setTo(black, mask);
            Core.inRange(hsvMat, lowerGreen, upperGreen, mask);
            input.setTo(black, mask);
            Core.inRange(hsvMat, lowerPurple, upperPurple, mask);
            input.setTo(black, mask);

            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 2);

            return input;
        }
    }
}