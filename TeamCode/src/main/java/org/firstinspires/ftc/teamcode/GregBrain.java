package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.tensorflow.lite.Interpreter;

import java.io.File;

@SuppressWarnings("unused")
@Autonomous(name = "Greg Brain") // Yeeeees we are getting the brain
public class GregBrain extends OpMode {
    @NonNull
    final
    ElapsedTime gametime = new ElapsedTime();
    @NonNull
    final
    RobotHardware robot = new RobotHardware();
    Interpreter brain;

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
        robot.hardwareLoop();
        float[] input_layer = new float[]{
                (float) gametime.seconds() / 30 // 30 second autonomous period
                , (float) robot.getPos().x
                , (float) robot.getPos().y
                , robot.getRot().thirdAngle // Heading
        };
        float[] output_layer = new float[4]; // Basic Joystick controls for now
        brain.run(input_layer, output_layer); // activate the  t h i n k i n g
        robot.chassis(new double[]{output_layer[0], output_layer[1], output_layer[2]});
        robot.performAction(decodeFloat(output_layer[3]));
    }

    @Override
    public void stop() {
        brain.close();
        robot.hardwareStop();
    }

    @NonNull
    private Action decodeFloat(float f) {
        double f_11 = f / 11.0;
        if (f_11 >= 0 && f_11 < 1) {
            return Action.OPEN_CLAW;
        }
        if (f_11 >= 1 && f_11 < 2) {
            return Action.CLOSE_CLAW;
        }
        if (f_11 >= 2 && f_11 < 3) {
            return Action.RETRACT_ARM;
        }
        if (f_11 >= 3 && f_11 < 4) {
            return Action.EXTEND_ARM;
        }
        if (f_11 >= 4 && f_11 < 5) {
            return Action.START_INTAKE;
        }
        if (f_11 >= 5 && f_11 < 6) {
            return Action.STOP_INTAKE;
        }
        if (f_11 >= 6 && f_11 < 7) {
            return Action.REVERSE_INTAKE;
        }
        if (f_11 >= 7 && f_11 < 8) {
            return Action.FIRE_LOW;
        }
        if (f_11 >= 8 && f_11 < 9) {
            return Action.FIRE_MID;
        }
        if (f_11 >= 9 && f_11 < 10) {
            return Action.FIRE_HIGH;
        }
        return Action.NONE;
    }


}
