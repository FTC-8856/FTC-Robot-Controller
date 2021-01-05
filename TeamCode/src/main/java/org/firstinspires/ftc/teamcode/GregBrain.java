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
                , (float) robot.get_pos().x
                , (float) robot.get_pos().y
                , robot.get_rot().thirdAngle // Heading
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
        double f_11 = f / 11.0;
        if (f_11 >= 0 && f_11 < 1) {
            return Action.OpenClaw;
        }
        if (f_11 >= 1 && f_11 < 2) {
            return Action.CloseClaw;
        }
        if (f_11 >= 2 && f_11 < 3) {
            return Action.RetractArm;
        }
        if (f_11 >= 3 && f_11 < 4) {
            return Action.ExtendArm;
        }
        if (f_11 >= 4 && f_11 < 5) {
            return Action.StartIntake;
        }
        if (f_11 >= 5 && f_11 < 6) {
            return Action.StopIntake;
        }
        if (f_11 >= 6 && f_11 < 7) {
            return Action.ReverseIntake;
        }
        if (f_11 >= 7 && f_11 < 8) {
            return Action.FireLow;
        }
        if (f_11 >= 8 && f_11 < 9) {
            return Action.FireMid;
        }
        if (f_11 >= 9 && f_11 < 10) {
            return Action.FireHigh;
        }
        return Action.None;
    }


}
