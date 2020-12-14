package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.tensorflow.lite.Interpreter;

import java.io.File;

@Autonomous(name="Greg Brain") // Yeeeees we are getting the brain
public class GregBrain extends OpMode {
    Interpreter brain;
    ElapsedTime gametime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    @Override
    public void init(){
        File tflite_file = new File("/FIRST/8856tensorflow/greg.tflite");
        brain = new Interpreter(tflite_file);
        robot.init(hardwareMap, "imu");
    }

    @Override
    public void start(){
        robot.startIMU(new Position(
                DistanceUnit.METER,
                0.0, 0.0, 0.0 // <--- starting position
                ,0
        ));
        gametime.reset();
    }

    @Override
    public void loop(){
        robot.hardware_loop();
        float[] input_layer = new float[]{
                 (float)gametime.seconds()/30 // 30 second autonomous period
                ,(float)robot.pos.x
                ,(float)robot.pos.y
                ,robot.rot.thirdAngle // Heading
        };
        float[] output_layer = new float[3]; // Basic Joystick controls for now
        brain.run(input_layer, output_layer); // activate the  t h i n k i n g
        setPower(robot.frontleft, output_layer[0], output_layer[1], output_layer[2]);
        setPower(robot.frontright, output_layer[0], output_layer[1], output_layer[2]);
        setPower(robot.backleft, output_layer[0], output_layer[1], output_layer[2]);
        setPower(robot.backright, output_layer[0], output_layer[1], output_layer[2]);
    }

    @Override
    public void stop(){
        brain.close();
        robot.hardware_stop();
    }

    private void setPower(@NonNull DcMotor motor, double right_stick_y, double right_stick_x, double left_stick_x) {
        Double[] values = robot.motorMap.get(motor);
        double forward_backward = values[0] * right_stick_y;
        double strafe = values[1] * right_stick_x;
        double turn = values[2] * left_stick_x;
        double power = forward_backward + strafe + turn;
        motor.setPower(Math.max(-1, Math.min(power, 1)));
    }
}
