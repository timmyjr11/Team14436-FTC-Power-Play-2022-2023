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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
@Autonomous
public class StateAutoRed extends LinearOpMode {

    //Cone 2 and 3 need side move

    private final FtcDashboard dash = FtcDashboard.getInstance();

    static ConfigPos.colors color = ConfigPos.colors.None;
    static ConfigPos.colors previousColor = ConfigPos.colors.None;


    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    int liftLevel = 170;
    int leftForward = 0;
    int leftSideLevel = 0;

    int rightForward = 0;
    int rightSideLevel = 0;

    int forward2 = 0;

    int coneCounter;

    ConfigPos.side side = ConfigPos.side.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean D1UpPressed;
    boolean D1DownPressed;

    boolean ready = false;


    public static int rectX = 127;
    public static int rectY = 45; //95 If right
    public static int rectHeight = 100;
    public static int rectWidth = 75;

    public static double lowerGreenH = 50;
    public static double lowerGreenS = 50;
    public static double lowerGreenV = 25;

    public static double lowerYellowH = 20;
    public static double lowerYellowS = 75;
    public static double lowerYellowV = 50;

    public static double lowerPurpleH = 120;
    public static double lowerPurpleS = 50;
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


    @SuppressWarnings("deprecation")
    @Override
    public void runOpMode() throws InterruptedException {

        d = new SampleMecanumDrive(hardwareMap);

        d.blueLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.blackLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        d.blueLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        d.blueLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.blackLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        d.blueGripper.setPosition(0);
        d.blackGripper.setPosition(0);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());


        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        telemetry.addLine("Opening Cameras...");
        telemetry.update();

        blueCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        blueCam.openCameraDevice();
        blackCam.openCameraDevice();

        telemetry.addLine("Cameras opened");
        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();

        while (true) {
            if (gamepad1.dpad_left) {
                d.setPoseEstimate(StatePoseStorage.leftAuto);
                side = ConfigPos.side.left;
                rectY = 45;
                blackCam.setPipeline(new ColorDetectionPipelineV2());
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
                dash.startCameraStream(blackCam, 30);
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(StatePoseStorage.rightAuto);
                rectY = 95;
                side = ConfigPos.side.right;
                blueCam.setPipeline(new ColorDetectionPipelineV2());
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dash.startCameraStream(blueCam, 30);
                break;
            }
            if (isStopRequested()) return;
        }

        while(!gamepad1.a) {
            telemetry.addData("Number of cones: ", coneCounter);
            telemetry.addLine("Press up on D-pad to add a cone, press down on D-pad to remove a cone");
            telemetry.addLine("Press A to move on to the next step");
            if (D1UpPressed && coneCounter < 5) {
                coneCounter++;
            } else if (D1DownPressed && coneCounter > 0) {
                coneCounter--;
            }
            D1UpPressed = ifPressed(gamepad1.dpad_up);
            D1DownPressed = ifPressed(gamepad1.dpad_down);
            booleanIncrementer = 0;
            telemetry.update();
            if(isStopRequested()) return;
        }

        TrajectorySequence leftSideV2 = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-34, -50, Math.toRadians(0)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    d.blackLift.setTargetPosition(1120);
                    d.blueLift.setTargetPosition(1120);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToConstantHeading(new Vector2d(-34, -20), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -3, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build(); // 9 x 8 y


        TrajectorySequence leftSideConesV2 = d.trajectorySequenceBuilder(leftSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -11.25, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-59.5, -11.25, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(-47, -11.25, Math.toRadians(90)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-28, -4, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        leftForward += 1;
        leftSideLevel -= 0.5;

        TrajectorySequence leftSideConesV2_2 = d.trajectorySequenceBuilder(leftSideConesV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-59.5, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(90)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-28 + leftSideLevel, -4 + leftForward, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        leftForward += 2.5;
        leftSideLevel -= 0.5;

        TrajectorySequence leftSideConesV2_3 = d.trajectorySequenceBuilder(leftSideConesV2_2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-59.5, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(90)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(-28 + leftSideLevel, -4 + leftForward, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        leftForward += 2;

        TrajectorySequence leftSideConesV2_4 = d.trajectorySequenceBuilder(leftSideConesV2_3.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-59.5, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(90)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-28, -4 + leftForward, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        leftForward += 2;

        TrajectorySequence leftSideConesV2_5 = d.trajectorySequenceBuilder(leftSideConesV2_4.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-59.5, -11.25 + leftForward, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(-47, -11.25 + leftForward, Math.toRadians(90)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(-28, -4 + leftForward, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();


        TrajectorySequence leftSideParkLeft = d.trajectorySequenceBuilder(leftSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-38, -20, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToConstantHeading(new Vector2d(-38, -30), Math.toRadians(270)) // If middle
                .lineToConstantHeading(new Vector2d(-12, -30))
                .build(); // If left

        TrajectorySequence leftSideParkMiddle = d.trajectorySequenceBuilder(leftSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-38, -20, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToConstantHeading(new Vector2d(-38, -30), Math.toRadians(270)) // If middle
                .build();

        TrajectorySequence leftSideParkRight = d.trajectorySequenceBuilder(leftSideConesV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-38, -20, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToConstantHeading(new Vector2d(-38, -30), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(-58, -30)) // If right
                .build();












        //TODO: RIGHT SIDE
        TrajectorySequence rightSideV2 = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(33.5, -50, Math.toRadians(180)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blueGripper.setPosition(0.8);
                    d.blackGripper.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    d.blackLift.setTargetPosition(1120);
                    d.blueLift.setTargetPosition(1120);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToConstantHeading(new Vector2d(33.5, -20), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(30, -4, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build(); // 9 x 8 y


        TrajectorySequence rightSideConesV2 = d.trajectorySequenceBuilder(rightSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -11.75, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(59.5, -11.75, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(47, -11.5, Math.toRadians(90)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(29, -5, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        rightForward += 0.5;
        rightSideLevel += 0.5;

        TrajectorySequence rightSideConesV2_2 = d.trajectorySequenceBuilder(rightSideConesV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -11.75 + rightForward, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(59.75, -11.75 + rightForward, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(47, -11.5 + rightForward, Math.toRadians(90)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1135);
                    d.blueLift.setTargetPosition(1135);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(30 + rightSideLevel, -5 + rightForward, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        rightForward += 1;
        rightSideLevel += 1;

        TrajectorySequence rightSideConesV2_3 = d.trajectorySequenceBuilder(rightSideConesV2_2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47, -11.75 + rightForward, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    d.blackLift.setTargetPosition(liftLevel);
                    d.blueLift.setTargetPosition(liftLevel);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToSplineHeading(new Pose2d(59.75, -11.75 + rightForward, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    d.blackGripper.setPosition(0.8);
                    d.blueGripper.setPosition(0.8);
                })
                .splineToSplineHeading(new Pose2d(47, -11.5 + rightForward, Math.toRadians(90)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    d.blackLift.setTargetPosition(1150);
                    d.blueLift.setTargetPosition(1150);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(1);
                    d.blueLift.setPower(1);
                })
                .splineToSplineHeading(new Pose2d(29.75 + rightSideLevel, -4.25 + rightForward, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(0.3)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2, () -> {
                    d.blackGripper.setPosition(0);
                    d.blueGripper.setPosition(0);
                })
                .build();

        TrajectorySequence rightSideParkLeft = d.trajectorySequenceBuilder(rightSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(38, -20, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToConstantHeading(new Vector2d(38, -32), Math.toRadians(270)) // If middle
                .lineToConstantHeading(new Vector2d(12, -32))
                .build(); // If left

        TrajectorySequence rightSideParkMiddle = d.trajectorySequenceBuilder(rightSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(38, -20, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToConstantHeading(new Vector2d(38, -32), Math.toRadians(270)) // If middle
                .build();

        TrajectorySequence rightSideParkRight = d.trajectorySequenceBuilder(rightSideV2.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(38, -20, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .splineToConstantHeading(new Vector2d(38, -32), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(58, -32)) // If right
                .build();

        ready = true;

        while (!isStarted()) {
            if (isStopRequested()) return;
        }

        switch (side) {
            case right:
                blueCam.stopStreaming();
                d.followTrajectorySequence(rightSideV2);
                if (coneCounter >= 1) { d.followTrajectorySequence(rightSideConesV2); }

                if (coneCounter >= 2) {
                    liftLevel -= 30;
                    d.followTrajectorySequence(rightSideConesV2_2);
                }

                if (coneCounter >= 3) {
                    liftLevel -= 30;
                    d.followTrajectorySequence(rightSideConesV2_3);
                }

                switch (color) {
                    case green:
                        d.followTrajectorySequence(rightSideParkLeft);
                        break;
                    case yellow:
                        d.followTrajectorySequence(rightSideParkMiddle);
                        break;
                    case purple:
                        d.followTrajectorySequence(rightSideParkRight);
                        break;
                }
                break;
            case left:
                blackCam.stopStreaming();
                d.followTrajectorySequence(leftSideV2);
                if (coneCounter >= 1 ) { d.followTrajectorySequence(leftSideConesV2); }

                if (coneCounter >= 2) {
                    liftLevel -= 30;
                    d.followTrajectorySequence(leftSideConesV2_2);
                }

                if (coneCounter >= 3) {
                    liftLevel -= 30;
                    d.followTrajectorySequence(leftSideConesV2_3);
                }

                if (coneCounter >= 4) {
                    liftLevel -= 30;
                    d.followTrajectorySequence(leftSideConesV2_4);
                }

                if (coneCounter >= 5) {
                    liftLevel -= 30;
                    d.followTrajectorySequence(leftSideConesV2_5);
                }

                switch (color) {
                    case green:
                        d.followTrajectorySequence(leftSideParkLeft);
                        break;
                    case yellow:
                        d.followTrajectorySequence(leftSideParkMiddle);
                        break;
                    case purple:
                        d.followTrajectorySequence(leftSideParkRight);

                        break;
                }
                break;
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

    @Config
    public class ColorDetectionPipelineV2 extends OpenCvPipeline {
        //Mat objects to hold the original image,
        // the filtered image, and the region of interest
        private final Mat hsvMat = new Mat();
        private final Mat mask = new Mat();
        private final Mat filtered = new Mat();
        private final Mat filteredG = new Mat();

        // Declare the rectangle to be used for processing
        Rect rect  = new Rect(rectX, rectY, rectWidth, rectHeight);

        // Declare the limits and colors needed for processing
        private final Scalar black = new Scalar(0, 0, 0);

        @Override
        public Mat processFrame(Mat input) {

            Scalar lowerGreen = new Scalar(lowerGreenH, lowerGreenS, lowerGreenV);
            Scalar upperGreen = new Scalar(upperGreenH, upperGreenS, upperGreenV);

            Scalar lowerYellow = new Scalar(lowerYellowH, lowerYellowS, lowerYellowV);
            Scalar upperYellow = new Scalar(upperYellowH, upperYellowS, upperYellowV);

            Scalar lowerPurple = new Scalar(lowerPurpleH, lowerPurpleS, lowerPurpleV);
            Scalar upperPurple = new Scalar(upperPurpleH, upperPurpleS, upperPurpleV);

            rect = new Rect(new Point(rectX, rectY), new Size(rectWidth, rectHeight));

            // Convert the color to HSV and create a submat for
            // Region of interest
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Mat roi = new Mat(hsvMat, rect);

            // Filter for yellow
            Core.inRange(roi, lowerYellow, upperYellow, mask);
            int yellowCount = Core.countNonZero(mask);

            // Filter for green
            Core.inRange(roi, lowerGreen, upperGreen, filteredG);
            int greenCount = Core.countNonZero(filteredG);

            // Filter for purple
            Core.inRange(roi, lowerPurple, upperPurple, filtered);
            int purpleCount = Core.countNonZero(filtered);

            // Combine the filters together and set extra colors to black
            Core.bitwise_or(mask, filteredG, mask);
            Core.bitwise_or(mask, filtered, mask);
            Core.bitwise_not(mask, mask);
            roi.setTo(black, mask);

            // Convert back to RGB and place the rectangle
            Imgproc.cvtColor(hsvMat, input, Imgproc.COLOR_HSV2RGB);
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 2);

            // Count the number of non-zero pixels in each of the masks
            if (greenCount > yellowCount && greenCount > purpleCount) {
                color = ConfigPos.colors.green;
                telemetry.addData("Color", color);
            } else if (yellowCount > greenCount && yellowCount > purpleCount) {
                color = ConfigPos.colors.yellow;
                telemetry.addData("Color", color);
            } else if (purpleCount > greenCount && purpleCount > yellowCount) {
                color = ConfigPos.colors.purple;
                telemetry.addData("Color", color);
            } else {
                color = ConfigPos.colors.None;
                telemetry.addData("Color", color);
            }

            telemetry.addData("Side", side);
            telemetry.addData("Cone count", coneCounter);
            telemetry.addData("Robot ready? ", ready);
            telemetry.addData("BLack cam FPS", blackCam.getFps());
            telemetry.addData("Blue cam FPS", blueCam.getFps());
            telemetry.update();
            return input;
        }
    }
}