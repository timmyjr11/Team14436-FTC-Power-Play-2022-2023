package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive d = new SampleMecanumDrive(hardwareMap);

        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        d.blackArm.setPosition(0);
        d.blueArm.setPosition(0);
        d.rotateServo.setPosition(0);
        d.blackGripper.setPosition(0);
        d.blueGripper.setPosition(0);

        while (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                power = 0.5;
            } else if (gamepad1.left_bumper) {
                power = 0.25;
            } else {
                power = 1;
            }

            d.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * power,
                            -gamepad1.left_stick_x * power,
                            -gamepad1.right_stick_x * power
                    )
            );

            d.update();

            Pose2d poseEstimate = d.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
