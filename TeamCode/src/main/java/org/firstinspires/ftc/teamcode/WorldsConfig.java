package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class WorldsConfig {

    public static Pose2d telePower = new Pose2d(0,0, Math.toRadians(180));
    public static Pose2d rightAuto = new Pose2d(37.5, -57.75, Math.toRadians(90));
    public static Pose2d leftAuto = new Pose2d(-37.5, -58, Math.toRadians(90));

    public enum gripper {
        closed,
        open
    }

    public enum rotation {
        upright,
        upsidedown
    }

    public enum arm {
        forward,
        backward
    }

    public enum side {
        left,
        right,
        tbd
    }

    public enum colors {
        green,
        purple,
        yellow,
        None
    }
}
