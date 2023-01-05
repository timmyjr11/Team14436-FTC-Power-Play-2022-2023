package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pastCodes.CactusPoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
@Autonomous
public class CrackedAuto extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    CompVisionForAuto CV = new CompVisionForAuto();


    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    int coneCounter;

    ConfigPos.side side = ConfigPos.side.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean D1UpPressed;
    boolean D1DownPressed;


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
                blueCam.setPipeline(CV);
                blueCam.pauseViewport();
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
                blackCam.setPipeline(CV);
                blackCam.pauseViewport();
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
                d.setPoseEstimate(PoseStorage.leftAuto);
                side = ConfigPos.side.left;
                blueCam.resumeViewport();
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(PoseStorage.rightAuto);
                side = ConfigPos.side.right;
                blackCam.resumeViewport();
                break;
            }
        }

        telemetry.clearAll();

        do {
            telemetry.addData("Number of cones: ", coneCounter);
            telemetry.addLine("Press up on D-pad to add a cone, press down on D-pad to remove a cone");
            telemetry.addLine("Press A to move on to the next step");
            telemetry.update();
            if (D1UpPressed) {
                coneCounter++;
            }
            if (D1DownPressed) {
                coneCounter--;
            }
            D1UpPressed = ifPressed(gamepad1.dpad_up);
            D1DownPressed = ifPressed(gamepad1.dpad_down);
            booleanIncrementer = 0;
        } while (!gamepad1.a);

        telemetry.clearAll();
        telemetry.addLine("Creating OpMode, please wait...");
        telemetry.addLine("Somebody once told me the world was gonna roll me, that I ain't the sharpest tool in the shed. - Abraham Lincoln");
        telemetry.update();

        TrajectorySequence leftSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    d.blueServo.setPosition(0.8);
                    d.blackServo.setPosition(0.8);
                })
                .lineToConstantHeading(new Vector2d(-37.5, -60))
                .lineToConstantHeading(new Vector2d(-12, -34))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blackLift.setTargetPosition(4000);
                    d.blueLift.setTargetPosition(4000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(-45))
                .forward(6)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blueServo.setPosition(0);
                    d.blackServo.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(-12, -34, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(-12, -12))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-35, -12))
                .build();
            TrajectorySequence leftGrabCones = d.trajectorySequenceBuilder(leftSide.end())
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        d.blackLift.setTargetPosition(800);
                        d.blueLift.setTargetPosition(800);
                        d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setPower(0.8);
                        d.blueLift.setPower(0.8);
                    })
                    .lineToConstantHeading(new Vector2d(-55, -12))
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                        d.blackServo.setPosition(0.8);
                        d.blueServo.setPosition(0.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        d.blackLift.setTargetPosition(4000);
                        d.blueLift.setTargetPosition(4000);
                        d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setPower(0.8);
                        d.blueLift.setPower(0.8);
                    })
                    .lineToConstantHeading(new Vector2d(-35, -12))
                    .turn(Math.toRadians(-135))
                    .forward(6)
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        d.blueServo.setPosition(0);
                        d.blackServo.setPosition(0);
                    })
                    .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                        d.blackLift.setTargetPosition(800);
                        d.blueLift.setTargetPosition(800);
                        d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setPower(0.8);
                        d.blueLift.setPower(0.8);
                    })
                    .build();

            TrajectorySequence LeftParking = d.trajectorySequenceBuilder(d.getPoseEstimate())
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        d.blackLift.setTargetPosition(0);
                        d.blueLift.setTargetPosition(0);
                        d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setPower(0.8);
                        d.blueLift.setPower(0.8);
                    })
                    .turn(90)
                    .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(270)))
                    .build();

                TrajectorySequence LeftParkingPosOne = d.trajectorySequenceBuilder(LeftParking.end())
                        .lineToConstantHeading(new Vector2d(-60, -35))
                        .build();
                TrajectorySequence LeftParkingPosTwo = d.trajectorySequenceBuilder(LeftParking.end())
                        .lineToConstantHeading(new Vector2d(-12, -35))
                        .build();

            telemetry.addData("Side Selected: ", side);
            telemetry.addData("Color: ", getColor());
            telemetry.update();
            waitForStart();

        if (side == ConfigPos.side.left) {
            d.followTrajectorySequence(leftSide);
            if (coneCounter > 0) {
                for (int i = 0; i < coneCounter; i++) {
                    d.followTrajectorySequence(leftGrabCones);
                }
            }
            d.followTrajectorySequence(LeftParking);
            if (getColor() == ConfigPos.colors.green) {
                d.followTrajectorySequence(LeftParkingPosOne);
            } else if (getColor() == ConfigPos.colors.purple) {
                d.followTrajectorySequence(LeftParkingPosTwo);
            }
        }



    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);

        //noinspection PointlessBooleanExpression
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }

    private ConfigPos.colors getColor() {
        return CV.getColor();
    }
}