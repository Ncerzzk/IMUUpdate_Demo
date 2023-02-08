package com.example.imu

import android.annotation.SuppressLint
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.content.Context
import android.widget.TextView
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.sqrt

class MainActivity : AppCompatActivity() {
    var acc:FloatArray = FloatArray(3)
    var gyro:FloatArray = FloatArray(3)

    val period = 2500

    var Q = FloatArray(4)

    val T:Float = 2.5f/1000.0f
    val KI:Float = 0.005f
    val KP:Float = 1.0f

    var roll:Float=0f
    var pitch:Float=0f

    var errInt = FloatArray(3)

    var rotationMatrix: FloatArray = FloatArray(9)
    var refAngles:FloatArray = FloatArray(3)
    var refQ:FloatArray = FloatArray(4)

    fun norm(src:FloatArray) : FloatArray{
        var norm = 0f
        val result = FloatArray(src.size)
        for(i in src.indices){
            norm += src[i] * src[i]
        }

        norm = sqrt(norm)
        for(i in src.indices){
            result[i] = src[i]/norm
        }

        return result
    }

    @SuppressLint("SetTextI18n")
    fun IMUUpdate(){
        val acc_n = norm(acc)
        var gyro_update = FloatArray(3)

        var acc_trans = FloatArray(3)
        var err = FloatArray(3)


        acc_trans[0] = 2*(Q[1]*Q[3]-Q[0]*Q[2])
        acc_trans[1] = 2*(Q[2]*Q[3]+Q[0]*Q[1])
        acc_trans[2] = Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3]

        err[0] = acc_n[1] * acc_trans[2] - acc_n[2] * acc_trans[1]
        err[1] = acc_n[2] * acc_trans[0] - acc_n[0] * acc_trans[2]
        err[2] = acc_n[0] * acc_trans[1] - acc_n[1] * acc_trans[0]

        for ( i in 0 until 3){
            errInt[i] += err[i]  * T
            gyro_update[i] = gyro[i] + err[i] * KP + errInt[i]*KI
        }

        Q[0] += T/2 * (-gyro_update[0]*Q[1]-gyro_update[1]*Q[2]-gyro_update[2]*Q[3])
        Q[1] += T/2 * ( gyro_update[0]*Q[0]-gyro_update[1]*Q[3]+gyro_update[2]*Q[2])
        Q[2] += T/2 * ( gyro_update[0]*Q[3]+gyro_update[1]*Q[0]-gyro_update[2]*Q[1])
        Q[3] += T/2 * (-gyro_update[0]*Q[2]+gyro_update[1]*Q[1]+gyro_update[2]*Q[0])

        //Q[0] += T/2 * (-gyro_update[0]*Q[1]-gyro_update[1]*Q[2]-gyro_update[2]*Q[3])
        //Q[1] += T/2 * ( gyro_update[0]*Q[0]+gyro_update[1]*Q[3]-gyro_update[2]*Q[2])
        //Q[2] += T/2 * (-gyro_update[0]*Q[3]+gyro_update[1]*Q[0]+gyro_update[2]*Q[1])
        //Q[3] += T/2 * ( gyro_update[0]*Q[2]-gyro_update[1]*Q[1]+gyro_update[2]*Q[0])

        Q = norm(Q)

        pitch =  asin(-2*(Q[2]*Q[3]-Q[0]*Q[1]))
        roll = atan2(2*(Q[1]*Q[3]+Q[0]*Q[2]),Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3])

        roll = roll * 180.0f/3.14159f
        pitch = pitch * 180.0f/3.14159f
        findViewById<TextView>(R.id.text).setText(
            "roll:$roll ref:${refAngles[2]} \npitch:$pitch ref:${refAngles[1]}\n" +
                    "q0:${Q[0]} ref:${refQ[0]}\n" +
                    "q1:${Q[1]} ref:${refQ[1]}\n" +
                    "q2:${Q[2]} ref:${refQ[2]}\n" +
                    "q3:${Q[3]} ref:${refQ[3]}\n"
        )

    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        val accSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyroSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val refSensor:Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)

        Q[0] = 1.0f // init Q[0] to 1
        Q[1] = 0.0f
        Q[2] = 0.0f
        Q[3] = 0.0f

        sensorManager.registerListener(object :SensorEventListener {
            override fun onSensorChanged(event: SensorEvent?) {
                if(event != null){
                    SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
                    SensorManager.getQuaternionFromVector(refQ,event.values)
                    SensorManager.getOrientation(rotationMatrix, refAngles)
                    refAngles=refAngles.map { it * 180.0f / 3.14159f }.toFloatArray()
                }
            }
            override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

        },refSensor,period)
        sensorManager.registerListener(object : SensorEventListener {
            override fun onSensorChanged(event: SensorEvent?) {
                if(event != null){
                    for(i in 0 until 3 ){
                        acc[i] = event.values[i]
                    }
                }
           }
            override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
        }, accSensor, period)


        sensorManager.registerListener(object : SensorEventListener {
            override fun onSensorChanged(event: SensorEvent?) {
                if(event!=null){
                    for(i in 0 until 3){
                        gyro[i] = event.values[i]
                    }
                    IMUUpdate()
                }
            }
            override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
        }, gyroSensor,period)

        
    }

}