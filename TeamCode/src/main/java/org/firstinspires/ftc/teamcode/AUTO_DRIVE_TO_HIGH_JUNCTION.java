package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "AUTO_DRIVE_TO_HIGH_JUNCTION", group = "Linear Opmode")
public class AUTO_DRIVE_TO_HIGH_JUNCTION extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(-38, 64, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        // go to high junction
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(35, 0), Math.toRadians(360))
                .build();

        drive.followTrajectory(traj);
/*

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 38), Math.toRadians(90))
                .forward(1)

                .build();
        drive.followTrajectory(traj2);

 */
    }
}
