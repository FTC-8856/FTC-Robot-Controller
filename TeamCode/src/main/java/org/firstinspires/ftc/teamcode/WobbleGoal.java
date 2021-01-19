package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@Autonomous(name = "Wobble Goal")
public class WobbleGoal extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "";
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
        double max = 0.0;
        Recognition maxRecognition = null;
        for (Recognition recognition : get_recognitions()) {
            if (recognition.getConfidence() > max) {
                maxRecognition = recognition;
            }
        }
        if (maxRecognition.getLabel().equals("zero")) {
            zero();
        } else if (maxRecognition.getLabel().equals("three")) {
            three();
        } else {
            four();
        }
    }


    private List<Recognition> get_recognitions() {
        if (tfod != null) {
            return tfod.getRecognitions();
        }
        return new ArrayList<>();
    }


    private void zero() {
        robot.closeClaw();
        robot.driveFor(88.0);
        robot.armPower(-1.0);
        robot.openClaw();
        robot.armPower(1.0);
    }

    private void three() {
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
