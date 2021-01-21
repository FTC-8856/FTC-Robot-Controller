package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@SuppressWarnings("unused")
@Autonomous(name = "Wobble Goal")
public class WobbleGoal extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "LyBonjnM^D@FB$fe4Y^vf^uRSjRLsWf8RVcHy%@BASDffQSLr8Tq!qHH&au2%S^vHGt%ZVsQyDv3m!ydfA$*Ye*";
    RobotHardware robot;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap, "");
        initVuforia();
        initTfod();
        waitForStart();
        goTo();
    }


    private void goTo() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions.isEmpty()) {
                tfod.shutdown();
                zero();
            } else {
                double oneConfidence = 0.0;
                double fourConfidence = 0.0;
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        oneConfidence = recognition.getConfidence();
                    } else if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                        fourConfidence = recognition.getConfidence();
                    }
                }
                if (fourConfidence > oneConfidence) {
                    telemetry.addData("Four, Confidence: %2f", fourConfidence);
                    telemetry.update();
                    tfod.shutdown();
                    four();
                } else {
                    telemetry.addData("One, Confidence: %2f", oneConfidence);
                    telemetry.update();
                    tfod.shutdown();
                    one();
                }
            }
        } else {
            telemetry.addData("no TensorFlow object detector found", null);
        }
    }


    private void zero() {
        robot.closeClaw();
        robot.driveFor(88.0);
        robot.armPower(-1.0);
        robot.openClaw();
        robot.armPower(1.0);
    }

    private void one() {
        robot.closeClaw();
        robot.driveFor(115.0);
        robot.armPower(-1.0);
        robot.openClaw();
        robot.armPower(1.0);
    }

    private void four() {
        robot.closeClaw();
        robot.driveFor(120.0);
        robot.armPower(1.0);
        robot.openClaw();
        robot.armPower(-1.0);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robot.getWebcam();
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
