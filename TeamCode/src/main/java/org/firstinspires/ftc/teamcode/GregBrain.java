package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.tensorflow.lite.Interpreter;

import java.io.File;

@Autonomous(name = "Greg Brain") // Yeeeees we are getting the brain
public class GregBrain extends OpMode {
    Interpreter brain;
    @NonNull
    ElapsedTime gametime = new ElapsedTime();
    @NonNull
    RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        File tflite_file = new File("/FIRST/8856tensorflow/greg.tflite");
        brain = new Interpreter(tflite_file);
        robot.init(hardwareMap, "imu");
    }

    @Override
    public void start() {
        robot.startIMU(new Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                , 0
        ));
        gametime.reset();
    }

    @Override
    public void loop() {
        robot.hardware_loop();
        float[] input_layer = new float[]{
                (float) gametime.seconds() / 30 // 30 second autonomous period
                , (float) robot.pos.x
                , (float) robot.pos.y
                , robot.rot.thirdAngle // Heading
        };
        float[] output_layer = new float[4]; // Basic Joystick controls for now
        brain.run(input_layer, output_layer); // activate the  t h i n k i n g
        robot.chassis(output_layer[0], output_layer[1], output_layer[2]);
        robot.performAction(decodeFloat(output_layer[4]));
    }

    @Override
    public void stop() {
        brain.close();
        robot.hardware_stop();
    }

    @NonNull
    private Action decodeFloat(float f) {
        if (f >= 0.125 && f < 0.25) {
            return Action.OpenClaw;
        }
        if (f >= 0.25 && f < 0.375) {
            return Action.CloseClaw;
        }
        if (f >= 0.375 && f < 0.5) {
            return Action.RetractArm;
        }
        if (f >= 0.5 && f < 0.625) {
            return Action.ExtendArm;
        }
        if (f >= 0.625 && f < 0.75) {
            return Action.StartIntake;
        }
        if (f >= 0.75 && f < 0.875) {
            return Action.StopIntake;
        }
        if (f >= 0.875 && f < 1.0) {
            return Action.ReverseIntake;
        }
        return Action.None;
    }


}
