package org.firstinspires.ftc.teamcode.testCodes.armAndLift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class armTest extends LinearOpMode {
    Servo blackArm;
    Servo blueArm;

    Servo blackGripper;
    Servo blueGripper;

    Servo rotateServo;



    @Override
    public void runOpMode() throws InterruptedException {
        blackArm = hardwareMap.get(Servo.class, "blackArm");
        blueArm = hardwareMap.get(Servo.class, "blueArm");
        blackGripper = hardwareMap.get(Servo.class, "blackGripper");
        blueGripper = hardwareMap.get(Servo.class, "blueGripper");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

        blueGripper.setDirection(Servo.Direction.REVERSE);
        blueArm.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        blackArm.setPosition(0.125);
        blueArm.setPosition(0.125);
        rotateServo.setPosition(0);
        blackGripper.setPosition(0);
        blueGripper.setPosition(0);

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                blackArm.setPosition(0);
                blueArm.setPosition(0);
                rotateServo.setPosition(0);
            }
            if (gamepad1.a) {
                blueArm.setPosition(0.83);
                blackArm.setPosition(0.83);
                rotateServo.setPosition(1);
            }

            if (gamepad1.dpad_up) rotateServo.setPosition(1);
            if (gamepad1.dpad_down) rotateServo.setPosition(0);

            if (gamepad1.dpad_left) {
                blackGripper.setPosition(0);
                blueGripper.setPosition(0);

            }

            if (gamepad1.dpad_right) {
                blackGripper.setPosition(1);
                blueGripper.setPosition(1);
            }
            telemetry.addData("Roate", rotateServo.getPosition());
            telemetry.addData("black arm", blackArm.getPosition());
            telemetry.addData("blue arm", blueArm.getPosition());
            telemetry.update();
        }
    }
}
