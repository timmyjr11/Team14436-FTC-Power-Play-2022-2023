package org.firstinspires.ftc.teamcode.testCodes;

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

        blackGripper.setDirection(Servo.Direction.REVERSE);
        blueArm.setDirection(Servo.Direction.REVERSE);
        rotateServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                blackArm.setPosition(0);
                blueArm.setPosition(0);
                rotateServo.setPosition(0);
            }
            if (gamepad1.a) {
                blueArm.setPosition(1);
                blackArm.setPosition(1);
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
        }
    }
}
