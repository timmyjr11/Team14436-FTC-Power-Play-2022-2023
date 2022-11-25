package org.firstinspires.ftc.teamcode.cameraCalibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class CalibrationUsingImages extends LinearOpMode {
    OpenCvCamera blueCam;
    OpenCvCamera blackCam;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        blueCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        blueCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                blueCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                blueCam.closeCameraDevice();
                telemetry.addLine("AA1");
                telemetry.addData("error", errorCode);

                telemetry.update();
            }
        });

        blackCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                blackCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                blackCam.closeCameraDevice();
                telemetry.addLine("AA2");
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(blueCam, 30);
        FtcDashboard.getInstance().startCameraStream(blackCam, 30);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            dash.getTelemetry();
        }
    }

        class UselessGreenBoxDrawingPipeline extends OpenCvPipeline
        {
            @Override
            public Mat processFrame(Mat input)
            {
                Imgproc.rectangle(
                        input,
                        new Point(
                                input.cols()/4,
                                input.rows()/4),
                        new Point(
                                input.cols()*(3f/4f),
                                input.rows()*(3f/4f)),
                        new Scalar(0, 255, 0), 4);

                return input;
            }
        }
}
