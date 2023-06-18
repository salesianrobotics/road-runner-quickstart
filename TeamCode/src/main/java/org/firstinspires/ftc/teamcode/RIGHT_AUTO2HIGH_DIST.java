package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name = "RIGHT_AUTO2HIGH_DIST", group = "Linear Opmode")

public class RIGHT_AUTO2HIGH_DIST extends AutonomousBot {
    private Blinker control_Hub;
    private Blinker expansion_Hub;
    private Servo gripperServo;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor linearSlide;
    private HardwareDevice logitechWebcam;
    private ColorSensor rEVColorSensorV3;
    private DistanceSensor rev2MDistanceSensor;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private Servo wristServo;


    // delivers to highest junction and returns true if successful

    private void deliverFirstCone() {
        rightStrafe(3.20,0.6);
        //sleep(300);
        // turnRight(45);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);
        sleep(SLEEP_TIME);
        moveForward(0.1, 0.5);
        sleep(SLEEP_TIME);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(SLEEP_TIME);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        //sleep(300);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        sleep(SLEEP_TIME);
        moveBackward(0.1, 0.5);
        //turnLeft(45);
        leftStrafe(0.55, 0.3);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        sleep(SLEEP_TIME);


    }


    private void deliverSecondCone(){

        turnRight(191);
        sleep(SLEEP_TIME);
        adjustLinearSlide(6.5);
        sleep(SLEEP_TIME);

        // go toward wall of the cone stack
        moveForward(1.0, 0.4);
        stopBeforeConeStackWall();
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        //sleep(300);
        raiseLinearSlide(HIGH_JUNCTION_POSITION);

        sleep(SLEEP_TIME);
        moveBackward(2.05, 0.5);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        leftStrafe(0.55, 0.3);
        moveForward(0.1, 0.5);
        sleep(SLEEP_TIME);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        //sleep(400);
        raiseLinearSlide(SLIGHT_DOWN_SLIDE_HIGH);
        sleep(SLEEP_TIME);
        stdGripperServo.setPosition(GRIPPER_OPENED_POSITION);
        sleep(SLEEP_TIME);
        moveBackward(0.1, 0.5);
        rightStrafe(0.5, 1.0);
    }



    private void Park1() {

        deliverFirstCone();
        deliverSecondCone();

       raiseLinearSlide(GROUND_JUNCTION_POSITION);

    }

    private void Park2() {

        deliverFirstCone();
        deliverSecondCone();
        moveForward(1.0, 1.0);
        raiseLinearSlide(GROUND_JUNCTION_POSITION);

    }

    private void Park3() {

        deliverFirstCone();

        deliverSecondCone();
        moveForward(2.0,0.9);
        raiseLinearSlide(GROUND_JUNCTION_POSITION);
    }

    @Override
    public void runOpMode() {

        init(hardwareMap);

        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        stdGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        sleep(1000);
        stdWristServo.setPosition(WRIST_REST_POSITION);
        //stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        telemetry.addData("Status", "Initialized");

        /////////////

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

       // adjustLinearSlide(6.5);
/*
        while(stdFrontDistanceSensor.getDistance(DistanceUnit.INCH) > 6.5){
            telemetry.addData("FrontDistanceSensor", "Distance is %5.2f", stdFrontDistanceSensor.getDistance(DistanceUnit.INCH) );
            telemetry.addData("DistanceSensorUnderGripper", "Distance is %5.2f", stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) );

            telemetry.update();
        }

        while(stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) < 6.5){
            telemetry.addData("DistanceSensorUnderGripper", "Distance is %5.2f", stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) );
            telemetry.update();
        }
*/
        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //double distanceFromGround = stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH);



        waitForStart();

        if (opModeIsActive()) {


            while (opModeIsActive()) {


                //scanFieldOfVision();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            // WRITE CODE WHAT TO DO WHEN DETECTED SIGNAL SLEEVE

                            if (recognition.getLabel().equals("SilverEagleSignal1") && recognition.getConfidence() * 100 > 0.750) {
                                Park1();
                                return;
                            } else if (recognition.getLabel().equals("SilverEagleSignal2") && recognition.getConfidence() * 100 > 0.750) {
                                Park2();
                                return;
                            } else if (recognition.getLabel().equals("SilverEagleSignal3") && recognition.getConfidence() * 100 > 0.750) {
                                Park3();
                                return;
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }
}
