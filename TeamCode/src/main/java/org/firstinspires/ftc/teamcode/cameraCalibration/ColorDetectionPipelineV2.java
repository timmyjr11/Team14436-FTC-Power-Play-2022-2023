package org.firstinspires.ftc.teamcode.cameraCalibration;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

/*
    TrajectorySequence rightSide = d.trajectorySequenceBuilder(d.getPoseEstimate())
            .splineToSplineHeading(new Pose2d(35, -47, Math.toRadians(0)), Math.toRadians(90))

            .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                d.blueGripper.setPosition(1);
                d.blackGripper.setPosition(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                d.blackLift.setTargetPosition(400);
                d.blueLift.setTargetPosition(400);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                d.blackLift.setTargetPosition(topJunction + 100);
                d.blueLift.setTargetPosition(topJunction + 100);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
                d.blueArm.setPosition(0.83);
                d.blackArm.setPosition(0.83);
                d.rotateServo.setPosition(1);
            })
            .splineToSplineHeading(new Pose2d(35, -20, Math.toRadians(0)), Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(32.1, -4.5, Math.toRadians(315)), Math.toRadians(135))
            .waitSeconds(0.5)
            .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                d.blackGripper.setPosition(0);
                d.blueGripper.setPosition(0);
            })
            .build(); // 9 x 8 y


    TrajectorySequence rightSideCones1 = d.trajectorySequenceBuilder(rightSide.end())
            // Grab first cone from stack
            .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                d.blueArm.setPosition(lowerGripperPos);
                d.blackArm.setPosition(lowerGripperPos);
                d.rotateServo.setPosition(0);
                d.blackLift.setTargetPosition(liftLevel);
                d.blueLift.setTargetPosition(liftLevel);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })

            .splineToSplineHeading(new Pose2d(59, -9, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                d.blackGripper.setPosition(1);
                d.blueGripper.setPosition(1);
            })
            // Place first cone
            .setReversed(true)
            .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(180))
            .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                d.blackLift.setTargetPosition(topJunction);
                d.blueLift.setTargetPosition(topJunction);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                d.blueArm.setPosition(0.83);
                d.blackArm.setPosition(0.83);
                d.rotateServo.setPosition(1);
            })
            .splineToSplineHeading(new Pose2d(31.5, -4.75, Math.toRadians(315)), Math.toRadians(135))
            .waitSeconds(0.2)
            .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                d.blackGripper.setPosition(0);
                d.blueGripper.setPosition(0);
            })
            .setReversed(false)
            .build();


    TrajectorySequence rightSideCones2 = d.trajectorySequenceBuilder(rightSideCones1.end())
            // Grab first cone from stack
            .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                d.blueArm.setPosition(lowerGripperPos);
                d.blackArm.setPosition(lowerGripperPos);
                d.rotateServo.setPosition(0);
                d.blackLift.setTargetPosition(liftLevel);
                d.blueLift.setTargetPosition(liftLevel);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })

            .splineToSplineHeading(new Pose2d(59, -9, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                d.blackGripper.setPosition(1);
                d.blueGripper.setPosition(1);
            })
            // Place first cone
            .setReversed(true)
            .splineToSplineHeading(new Pose2d(47, -9, Math.toRadians(0)), Math.toRadians(180))
            .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                d.blackLift.setTargetPosition(topJunction);
                d.blueLift.setTargetPosition(topJunction);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                d.blueArm.setPosition(0.83);
                d.blackArm.setPosition(0.83);
                d.rotateServo.setPosition(1);
            })
            .splineToSplineHeading(new Pose2d(31.5, -6, Math.toRadians(315)), Math.toRadians(135))
            .waitSeconds(0.2)
            .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                d.blackGripper.setPosition(0);
                d.blueGripper.setPosition(0);
            })
            .setReversed(false)
            .build();

    TrajectorySequence rightSideCones3 = d.trajectorySequenceBuilder(rightSideCones2.end())
            // Grab first cone from stack
            .splineToSplineHeading(new Pose2d(47, -10, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                d.blueArm.setPosition(lowerGripperPos);
                d.blackArm.setPosition(lowerGripperPos);
                d.rotateServo.setPosition(0);
                d.blackLift.setTargetPosition(liftLevel);
                d.blueLift.setTargetPosition(liftLevel);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })

            .splineToSplineHeading(new Pose2d(59, -10, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                d.blackGripper.setPosition(1);
                d.blueGripper.setPosition(1);
            })
            // Place first cone
            .setReversed(true)
            .splineToSplineHeading(new Pose2d(47, -10, Math.toRadians(0)), Math.toRadians(180))
            .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                d.blackLift.setTargetPosition(topJunction);
                d.blueLift.setTargetPosition(topJunction);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                d.blueArm.setPosition(0.83);
                d.blackArm.setPosition(0.83);
                d.rotateServo.setPosition(1);
            })
            .splineToSplineHeading(new Pose2d(31.5, -6.75, Math.toRadians(315)), Math.toRadians(135))
            .waitSeconds(0.2)
            .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                d.blackGripper.setPosition(0);
                d.blueGripper.setPosition(0);
            })
            .setReversed(false)
            .build();


    TrajectorySequence rightSideCones4 = d.trajectorySequenceBuilder(rightSideCones3.end())
            // Grab first cone from stack
            .splineToSplineHeading(new Pose2d(47, -11, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                d.blueArm.setPosition(lowerGripperPos);
                d.blackArm.setPosition(lowerGripperPos);
                d.rotateServo.setPosition(0);
                d.blackLift.setTargetPosition(liftLevel);
                d.blueLift.setTargetPosition(liftLevel);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })

            .splineToSplineHeading(new Pose2d(59, -11, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                d.blackGripper.setPosition(1);
                d.blueGripper.setPosition(1);
            })
            // Place first cone
            .setReversed(true)
            .splineToSplineHeading(new Pose2d(47, -11, Math.toRadians(0)), Math.toRadians(180))
            .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                d.blackLift.setTargetPosition(topJunction);
                d.blueLift.setTargetPosition(topJunction);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                d.blueArm.setPosition(0.83);
                d.blackArm.setPosition(0.83);
                d.rotateServo.setPosition(1);
            })
            .splineToSplineHeading(new Pose2d(31.5, -7, Math.toRadians(315)), Math.toRadians(135))
            .waitSeconds(0.2)
            .UNSTABLE_addDisplacementMarkerOffset(-0.05, () -> {
                d.blackGripper.setPosition(0);
                d.blueGripper.setPosition(0);
            })
            .setReversed(false)
            .build();

    TrajectorySequence rightSideCones5 = d.trajectorySequenceBuilder(rightSideCones4.end())
            // Grab first cone from stack
            .splineToSplineHeading(new Pose2d(47, -11.5, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                d.blueArm.setPosition(lowerGripperPos);
                d.blackArm.setPosition(lowerGripperPos);
                d.rotateServo.setPosition(0);
                d.blackLift.setTargetPosition(liftLevel);
                d.blueLift.setTargetPosition(liftLevel);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })

            .splineToSplineHeading(new Pose2d(59, -11.5, Math.toRadians(0)), Math.toRadians(0))
            .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                d.blackGripper.setPosition(1);
                d.blueGripper.setPosition(1);
            })
            // Place first cone
            .setReversed(true)
            .splineToSplineHeading(new Pose2d(47, -11.5, Math.toRadians(0)), Math.toRadians(180))
            .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                d.blackLift.setTargetPosition(topJunction);
                d.blueLift.setTargetPosition(topJunction);
                d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                d.blackLift.setPower(1);
                d.blueLift.setPower(1);
            })
            .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                d.blueArm.setPosition(0.83);
                d.blackArm.setPosition(0.83);
                d.rotateServo.setPosition(1);
            })
            .splineToSplineHeading(new Pose2d(31.5, -7, Math.toRadians(315)), Math.toRadians(135))
            .waitSeconds(0.2)
            .UNSTABLE_addDisplacementMarkerOffset(-0.1, () -> {
                d.blackGripper.setPosition(0);
                d.blueGripper.setPosition(0);
            })
            .setReversed(false)
            .build();*/
