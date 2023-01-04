package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pastCodes.CactusPoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class CrackedAuto extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    Pose2d start;

    int coneCounter;

    ConfigPos.side side = ConfigPos.side.tbd;


    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);

        d.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        d.blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        d.blueServo.setPosition(0);
        d.blackServo.setPosition(0);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        blueCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        blueCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                blackCam.closeCameraDevice();
                telemetry.addLine("AA2");
                telemetry.addData("error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                d.setPoseEstimate(CactusPoseStorage.leftAutoRed);
                side = ConfigPos.side.left;
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(CactusPoseStorage.rightAutoRed);
                side = ConfigPos.side.right;
                break;
            }
        }

        telemetry.clearAll();

        while (true) {
            telemetry.addLine("")
        }

        telemetry.clearAll();
        telemetry.addLine("Creating OpMode, please wait...");
        telemetry.addLine("Somebody once told me the world was gonna roll me, that I ain't the sharpest tool in the shed. - Abraham Lincoln");
        telemetry.update();

    }
}