package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

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
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
@Autonomous
public class CrackedAuto extends LinearOpMode {

    private final FtcDashboard dash = FtcDashboard.getInstance();

    static ConfigPos.colors color = ConfigPos.colors.None;


    SampleMecanumDrive d;

    OpenCvWebcam blueCam;
    OpenCvWebcam blackCam;

    int coneCounter;

    ConfigPos.side side = ConfigPos.side.tbd;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer = 0;
    boolean D1UpPressed;
    boolean D1DownPressed;


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

        d.blueServo.setPosition(0);
        d.blackServo.setPosition(0);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        telemetry.setMsTransmissionInterval(250);

        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());


        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId,
                        2,
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        telemetry.addLine("Opening Cameras...");
        telemetry.update();
        dash.getTelemetry();

        blueCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        blackCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        blueCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        blackCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);


        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();
        dash.getTelemetry();
        while (true) {
            if (gamepad1.dpad_left) {
                telemetry.addLine("Opening Cameras...");
                telemetry.update();
                dash.getTelemetry();
                d.setPoseEstimate(PoseStorage.leftAuto);
                side = ConfigPos.side.left;
                blackCam.openCameraDevice();
                blackCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
              //  blackCam.setPipeline(new CompVisionForAuto());
                dash.startCameraStream(blackCam, 30);
                break;
            } else if (gamepad1.dpad_right) {
                telemetry.addLine("Opening Cameras...");
                telemetry.update();
                dash.getTelemetry();
                d.setPoseEstimate(PoseStorage.rightAuto);
                side = ConfigPos.side.right;
                blueCam.openCameraDevice();
                blueCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
               // blueCam.setPipeline(new CompVisionForAuto());
                dash.startCameraStream(blueCam, 30);
                break;
            }
        }


        while(!gamepad1.a) {
            telemetry.addData("Number of cones: ", coneCounter);
            telemetry.addLine("Press up on D-pad to add a cone, press down on D-pad to remove a cone");
            telemetry.addLine("Press A to move on to the next step");
            telemetry.update();
            dash.getTelemetry();
            if (D1UpPressed) {
                coneCounter++;
            }
            if (D1DownPressed) {
                coneCounter--;
            }
            D1UpPressed = ifPressed(gamepad1.dpad_up);
            D1DownPressed = ifPressed(gamepad1.dpad_down);
            booleanIncrementer = 0;
        }

        telemetry.addLine("Creating OpMode, please wait...");
        telemetry.addLine("Somebody once told me the world was gonna roll me, that I ain't the sharpest tool in the shed. - Abraham Lincoln");
        telemetry.update();
        dash.getTelemetry();

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

        while (!isStarted()) {
            telemetry.addData("Color" , color);
            if (color == ConfigPos.colors.green) {
                telemetry.addLine("Color is green");
            } else if (color == ConfigPos.colors.purple) {
                telemetry.addLine("Color is purple");
            } else if (color == ConfigPos.colors.yellow) {
                telemetry.addLine("Color is Yellow");
            } else {
                telemetry.addLine("There is no color");
            }

            telemetry.addData("Side", side);
            telemetry.addData("Cone count", coneCounter);
            telemetry.update();
            dash.getTelemetry();
            if (isStopRequested()) return;
            if (isStarted()) break;
        }

        if (side == ConfigPos.side.left) {
            d.followTrajectorySequence(leftSide);
            if (coneCounter > 0) {
                for (int i = 0; i < coneCounter; i++) {
                    d.followTrajectorySequence(leftGrabCones);
                }
            }
            d.followTrajectorySequence(LeftParking);
            if (color == ConfigPos.colors.green) {
                d.followTrajectorySequence(LeftParkingPosOne);
            } else if (color == ConfigPos.colors.purple) {
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

    static class CompVisionForAuto extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat mask = new Mat();



        public int rectX = 0;
        public int rectY = 0;
        public int rectHeight = 0;
        public int rectWidth = 0;

        public double lowerGreenH = 50;
        public double lowerGreenS = 50;
        public double lowerGreenV = 25;

        public double lowerYellowH = 20;
        public double lowerYellowS = 200;
        public double lowerYellowV = 50;

        public double lowerPurpleH = 120;
        public double lowerPurpleS = 0;
        public double lowerPurpleV = 0;

        public double upperGreenH = 90;
        public double upperGreenS = 255;
        public double upperGreenV = 255;

        public double upperYellowH = 35;
        public double upperYellowS = 255;
        public double upperYellowV = 255;

        public double upperPurpleH = 170;
        public double upperPurpleS = 255;
        public double upperPurpleV = 255;

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
    }
}