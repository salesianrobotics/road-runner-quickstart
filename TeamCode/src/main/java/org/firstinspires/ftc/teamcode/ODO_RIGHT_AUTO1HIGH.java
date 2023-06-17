package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "ODO_RIGHT_AUTO1HIGH", group = "Linear Opmode")

public class ODO_RIGHT_AUTO1HIGH extends SmartBot {

    // delivers to highest junction and returns true if successful


    public ODO_RIGHT_AUTO1HIGH() {


    }
    private void Park1() {

        rightStrafe(3.25,0.6);
        sleep(300);
       // turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(200);
        //moveForward(0.1, 0.5);
        //sleep(200);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(200);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(200);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
        leftStrafe(0.55, 0.3);
        sleep(200);
        turnRight(191);
        sleep(200);
        adjustLinearSlide(6.5);
        sleep(300);

        moveForward(1.0, 0.4);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(300);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);

        sleep(200);
        moveBackward(2.05, 0.5);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        leftStrafe(0.50, 0.3);
        //moveForward(0.1, 0.5);
        sleep(200);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        //sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(200);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(200);
        moveBackward(0.1, 0.5);
        rightStrafe(0.5, 1.0);
        //leftStrafe(1.1, 0.3);
       // raiseLinearSlide(GROUND_JUNCTION_POSITION);



        // stdWristServo.setPosition(WRIST_UP_POSITION);

    }

    private void Park2() {

        rightStrafe(3.35,0.3);
        sleep(300);
        //turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(300);
        moveForward(0.1, 0.5);
        sleep(400);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(400);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
       // moveBackward(0.2, 0.5);
        leftStrafe(0.5, 0.3);
        sleep(200);
        turnRight(191);
        sleep(200);
        // raiseLinearSlide(GROUND_POSITION);
        //stdLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(400);
        raiseLinearSlide(AUTO_CONE_POSITION);
        sleep(300);

        moveForward(1.0, 0.4);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        //leftStrafe(1.1, 0.3);
        // raiseLinearSlide(GROUND_JUNCTION_POSITION);
    }

    private void Park3() {


        rightStrafe(3.35,0.3);
        sleep(300);
        //turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(300);
        moveForward(0.1, 0.5);
        sleep(400);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(400);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
        leftStrafe(0.5, 0.3);
        sleep(200);
        turnRight(191);
        sleep(200);
        // raiseLinearSlide(GROUND_POSITION);
        //stdLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(400);
        raiseLinearSlide(AUTO_CONE_POSITION);
        sleep(300);

        moveForward(1.0, 0.4);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(300);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(200);
        moveBackward(2.0, 0.5);
        leftStrafe(0.4, 0.3);
        moveForward(0.1, 0.5);
        sleep(200);
        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        //sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(200);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(200);
        moveBackward(0.1, 0.5);
        leftStrafe(0.5, 0.5);
        // raiseLinearSlide(GROUND_JUNCTION_POSITION);
    }

    @Override
    public void runOpMode() throws  InterruptedException {

        initRoadRunner(hardwareMap);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        String signalZone = getSignalZone();
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(1000);
        stdWristServo.setPosition(WRIST_REST_POSITION);

        // close gripper
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(1000);

        // move gripper back to rest position
        stdWristServo.setPosition(WRIST_REST_POSITION);

        telemetry.addData("Status", "Initialized");

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        if (signalZone.equals(""))
            signalZone = getSignalZone();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));

        stdMecanumDrive.setPoseEstimate(startPose);

        Trajectory traj1= stdMecanumDrive.trajectoryBuilder(startPose)
                .strafeRight(64)
                .addDisplacementMarker(() -> {
                    raiseLinearSlide(HIGH_JUNCTION_POSITION);
                })
                .build();

        //deliver cone
        Trajectory traj2 = stdMecanumDrive.trajectoryBuilder(traj1.end())
                .forward(2)
                .addDisplacementMarker(() -> {
                    stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
                    raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
                    stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
                    stdWristServo.setPosition(WRIST_REST_POSITION);
                })
                .build();

        // move back 2 inches
        Trajectory traj3 = stdMecanumDrive.trajectoryBuilder(traj2.end())
                .back(2)
                .build();

        Trajectory traj4 = stdMecanumDrive.trajectoryBuilder(traj3.end())
                .strafeLeft(6)
                .addDisplacementMarker(() ->
                {
                    stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
                    adjustLinearSlide(6.5);
                })
                .build();

        stdMecanumDrive.followTrajectory(traj1);
        stdMecanumDrive.followTrajectory(traj2);

        stdMecanumDrive.followTrajectory(traj3);
        stdMecanumDrive.followTrajectory(traj4);
        stdMecanumDrive.turn(Math.toRadians(180));
    }
}
