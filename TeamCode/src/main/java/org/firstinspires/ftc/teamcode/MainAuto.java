package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class MainAuto extends LinearOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive d;
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

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        d.blueServo.setPosition(0);
        d.blackServo.setPosition(0);

        telemetry.addLine("Press left on D-pad for left side, press right on D-pad for right side");
        telemetry.update();
        while (true) {
            if (gamepad1.dpad_left) {
                d.setPoseEstimate(PoseStorage.leftAutoRed);
                side = ConfigPos.side.left;
                break;
            } else if (gamepad1.dpad_right) {
                d.setPoseEstimate(PoseStorage.rightAutoRed);
                side = ConfigPos.side.right;
                break;
            }
        }

        telemetry.clearAll();

        telemetry.addLine("Creating OpMode, please wait...");
        telemetry.addLine("Somebody once told me the world was gonna roll me, that I ain't the sharpest tool in the shed. - Abraham Lincoln");
        telemetry.update();

            TrajectorySequence redSideRight = d.trajectorySequenceBuilder(d.getPoseEstimate())
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        d.blueServo.setPosition(1);
                        d.blackServo.setPosition(1);
                    })
                    .waitSeconds(0.3)
                    .strafeTo(new Vector2d(9, -59))
                    .lineToLinearHeading(new Pose2d(9, -10, Math.toRadians(90)))
                    .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                        d.blackLift.setTargetPosition(4000);
                        d.blueLift.setTargetPosition(4000);
                        d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setPower(0.8);
                        d.blueLift.setPower(0.8);
                    })
                    .turn(Math.toRadians(-40))
                    .forward(8.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        d.blueServo.setPosition(0);
                        d.blackServo.setPosition(0);
                    })
                    .waitSeconds(1)
                    .back(8.5)
                    .turn(Math.toRadians(40))
                    .lineToLinearHeading(new Pose2d(9, -60, Math.toRadians(90)))
                    .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                        d.blackLift.setTargetPosition(0);
                        d.blueLift.setTargetPosition(0);
                        d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        d.blackLift.setPower(0.8);
                        d.blueLift.setPower(0.8);
                    })
                    .turn(Math.toRadians(90))
                    .build();


        TrajectorySequence redSideLeft = d.trajectorySequenceBuilder(d.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    d.blueServo.setPosition(1);
                    d.blackServo.setPosition(1);
                })
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(-4, -59))
                .lineToLinearHeading(new Pose2d(-4, -10, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    d.blackLift.setTargetPosition(4000);
                    d.blueLift.setTargetPosition(4000);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(47))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    d.blueServo.setPosition(0);
                    d.blackServo.setPosition(0);
                })
                .waitSeconds(1)
                .back(7)
                .turn(Math.toRadians(-47))
                .lineToLinearHeading(new Pose2d(-4, -60, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    d.blackLift.setTargetPosition(0);
                    d.blueLift.setTargetPosition(0);
                    d.blueLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    d.blackLift.setPower(0.8);
                    d.blueLift.setPower(0.8);
                })
                .turn(Math.toRadians(-90))
                .build();

        telemetry.clearAll();

        telemetry.addLine("Side selected:");
        if (side == ConfigPos.side.left) {
            telemetry.addLine("Left side selected!");
        } else if (side == ConfigPos.side.right) {
            telemetry.addLine("Right side selected!");
        }

        telemetry.addLine("If there is anything wrong please restart!");
        telemetry.update();

        if(isStopRequested()) {
            return;
        }

        waitForStart();

        if(side == ConfigPos.side.left) {
            d.followTrajectorySequence(redSideLeft);
        } else if (side == ConfigPos.side.right) {
            d.followTrajectorySequence(redSideRight);
        }
    }
}
