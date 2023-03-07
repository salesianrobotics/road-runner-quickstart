/********* SILVER EAGLE 2022-2023 FTC POWERPLAY **************/

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class AutonomousBot extends StandardBot {
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    static final String TFOD_MODEL_FILE = "SilverEagleSignalSleeveV1.0.tflite";
    static final String[] LABELS = {
            "SilverEagleSignal1",
            "SilverEagleSignal2",
            "SilverEagleSignal3"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    static final String VUFORIA_KEY =
            "AZOImLb/////AAABmfzn2uj5DUp8hXLqjZPI08tknrCi8i5Bh3EO5hAASEpOVIbaLggw6xHMN4vGzLS13kmV35Z1bnE9stQafvASuvzPJfHZLXbNegPkIfVqggwvOsoiLItss41X8RiitJE1OTQafU80VtIR93FplVLwAl3/hrLMUz0HRIAJGRB13mx7wHo6TTgvOySzpqDCT3VezG+iHtyXuT749QNbwkosHgwheD9I3yMDOE0bxcdcuFwzurDz2rB3cCttvn4Vfpmlyfn9vmiBJ8pBtW1Nn5DUJ3ab59e5CXk6SFtKixRzbjZ5/XhSR48GKiV74knMED343ST6AV02Aju0cupfflG+g7okhNX7QvZ0Bi7N4vYlo9m0";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    TFObjectDetector tfod;

    StandardTrackingWheelLocalizer localizer;
    @Override
    public void runOpMode() {

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "LogitechWebcamRight");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = MIN_DETECTION_CONFIDENCE;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    void initLocalizer()
    {
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }

    void correctPathHeading(Pose2d currentPose, Pose2d expectedPose)
    {
        double headingDifference = currentPose.getHeading() - expectedPose.getHeading();
        double xDifference = currentPose.getX() - expectedPose.getX();
        double yDifference = currentPose.getY() - expectedPose.getY();

        turnRight(Math.toRadians(headingDifference));
        leftStrafeInches(xDifference);
        moveForwardInches(yDifference);

        telemetry.addData("Path-Heading Correction", "X-Difference (%.2f)", xDifference);
        telemetry.addData("Path-Heading Correction", "Y-Difference (%.2f)", yDifference);
        telemetry.addData("Path-Heading Correction", "Heading-Difference (%.2f)", headingDifference);

    }

    void scanFieldOfVision() {
        leftStrafe(.1, .5);
        rightStrafe(.1, .5);
    }
}
