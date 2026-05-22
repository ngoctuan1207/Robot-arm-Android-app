package com.example.RobotArm

import android.Manifest
import android.app.Dialog
import android.content.Context
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.os.Environment
import android.os.Handler
import android.os.Looper
import android.text.Editable
import android.text.TextWatcher
import android.util.Log
import android.view.MotionEvent
import android.view.TextureView
import com.google.gson.Gson
import android.view.View
import android.view.Window
import android.view.WindowManager
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.ImageButton
import android.widget.ImageView
import android.widget.SeekBar
import android.widget.Spinner
import android.widget.TextView
import android.widget.Toast
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.lifecycle.lifecycleScope
import io.github.controlwear.virtual.joystick.android.JoystickView
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.IOException
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.round
import kotlin.math.sin
import kotlin.system.exitProcess


class MainActivity : AppCompatActivity() {

    object CommandFlags {
        const val MoveJoystick: Byte    = 0x09
        const val Move: Byte            = 0x10
        const val Calib: Byte           = 0x11
        const val EmerStop: Byte        = 0x12
        const val Get_angle: Byte       = 0x13
        const val Standby: Byte         = 0x14
        const val Get_all: Byte         = 0x15
        const val Stop: Byte            = 0x16
        const val Move_Offset: Byte     = 0x17 //Used when robot lost calibration
        const val MoveHome: Byte        = 0x26
        const val ClearError: Byte      = 0x27
//        const val Rob_ena: Byte         = 0x29
    }

    object StatusFlags {
        const val Calibrated: Byte      = 0x01
        const val Busy: Byte            = 0x02
        const val Error: Byte           = 0x03
        const val Complete: Byte        = 0x28
        const val Home: Byte            = 0x25      // Out of Range
        const val OOR: Byte             = 0x04      // Out of Range
        const val OOS: Byte             = 0x05      // Out of Speed
        const val LIM: Byte             = 0x06      // Lost I2C Memory
        const val LIE: Byte             = 0x07      // Lost I2C Encoder
        const val LI: Byte              = 0x08      // Lost all I2C Peripheral
        const val TCP_error: Byte       = 0x0A
        const val TCP_norm: Byte        = 0x0B
    }

    object JogMode {
        const val Linear: Byte          = 0x18
        const val J123: Byte            = 0x19
        const val J456: Byte            = 0x20
        const val Reori: Byte           = 0x21
        const val Angle: Byte           = 0x22
        const val Posi: Byte            = 0x23
        const val Calib: Byte           = 0x24
    }

    private lateinit var joystick1: JoystickView
    private lateinit var joystick2: JoystickView
    private lateinit var btnEnable: ImageButton
    private lateinit var btnEstop: ImageButton
    private lateinit var btnCalib: ImageButton
    private lateinit var btnMove: ImageButton
    private lateinit var btnSolve: ImageButton
    private lateinit var btnHome: ImageButton
    private lateinit var btnMoveLinearZ: ImageButton
    private lateinit var btnMoveLinearY: ImageButton
    private lateinit var btnMoveLinearX: ImageButton
    private lateinit var btnMoveOrientZ: ImageButton
    private lateinit var btnMoveOrientY: ImageButton
    private lateinit var btnMoveOrientX: ImageButton

    private lateinit var btn_Zp: ImageButton
    private lateinit var btn_Yp: ImageButton
    private lateinit var btn_Xp: ImageButton
    private lateinit var btn_Zm: ImageButton
    private lateinit var btn_Ym: ImageButton
    private lateinit var btn_Xm: ImageButton


    private lateinit var speedBar: SeekBar
    private lateinit var speedView: TextView
    private lateinit var accelBar: SeekBar
    private lateinit var accelView: TextView
    private lateinit var spinnerJogMode: Spinner
    private lateinit var client: ESP32_Server
    private lateinit var imageView_J1_HomeFlag: ImageView
    private lateinit var imageView_J2_HomeFlag: ImageView
    private lateinit var imageView_J3_HomeFlag: ImageView
    private lateinit var imageView_J4_HomeFlag: ImageView
    private lateinit var imageView_J5_HomeFlag: ImageView
    private lateinit var imageView_J6_HomeFlag: ImageView

    //--------------------------Begin of Angle------------------------------
    private lateinit var sBarJ1_angle: SeekBar
    private lateinit var txtViewJ1_tarAngle: TextView
    private lateinit var sBarJ2_angle: SeekBar
    private lateinit var txtViewJ2_tarAngle: TextView
    private lateinit var sBarJ3_angle: SeekBar
    private lateinit var txtViewJ3_tarAngle: TextView
    private lateinit var sBarJ4_angle: SeekBar
    private lateinit var txtViewJ4_tarAngle: TextView
    private lateinit var sBarJ5_angle: SeekBar
    private lateinit var txtViewJ5_tarAngle: TextView
    private lateinit var sBarJ6_angle: SeekBar
    private lateinit var txtViewJ6_tarAngle: TextView


    private lateinit var sBarEZ_angle: SeekBar
    private lateinit var txtViewEZ_tarAngle: TextView
    private lateinit var sBarEY_angle: SeekBar
    private lateinit var txtViewEY_tarAngle: TextView
    private lateinit var sBarEX_angle: SeekBar
    private lateinit var txtViewEX_tarAngle: TextView

    private lateinit var txtView_J1_cur_angle: TextView
    private lateinit var txtView_J2_cur_angle: TextView
    private lateinit var txtView_J3_cur_angle: TextView
    private lateinit var txtView_J4_cur_angle: TextView
    private lateinit var txtView_J5_cur_angle: TextView
    private lateinit var txtView_J6_cur_angle: TextView
    private lateinit var txtView_X_cur: TextView
    private lateinit var txtView_Y_cur: TextView
    private lateinit var txtView_Z_cur: TextView
    private lateinit var txtView_EX_cur: TextView
    private lateinit var txtView_EY_cur: TextView
    private lateinit var txtView_EZ_cur: TextView

    private lateinit var txtviewJ1: TextView
    private lateinit var txtviewJ2: TextView
    private lateinit var txtviewJ3: TextView
    private lateinit var txtviewJ4: TextView
    private lateinit var txtviewJ5: TextView
    private lateinit var txtviewJ6: TextView
    private lateinit var txtviewX: TextView
    private lateinit var txtviewY: TextView
    private lateinit var txtviewZ: TextView
    private lateinit var txtviewEX: TextView
    private lateinit var txtviewEY: TextView
    private lateinit var txtviewEZ: TextView
    private lateinit var editTxt_X_tar: EditText
    private lateinit var editTxt_Y_tar: EditText
    private lateinit var editTxt_Z_tar: EditText
    private lateinit var unityContainer:TextureView

    private var hasNewData = false
    private var uiReady = false
    private var lastRobStat: Int? = null
    private var isErrorDialogShowing = false
    private var isRobError = false
    private var isConnected = false
    private var isTarChanged = false
    private var lastConnectionState: Boolean? = null
    private var ikDialog: Dialog? = null
    //--------------------------End of Angle--------------------------------

    private val esp32_Ip = "192.168.4.1"
    private val esp32_Port = 80
    //Sampling value for Joystick
    val sampleRate = 500   // ms
    private var Joystick1Moving: Boolean = false
    private var Joystick2Moving: Boolean = false

    private val max_angle_Values = floatArrayOf(0.0f, 180.0f, 110.0f, 70.0f, 120.0f, 130.0f, 180.0f)
    private val min_angle_Values = floatArrayOf(0.0f, -180.0f, -110.0f, -110.0f, -120.0f, -120.0f, -180.0f)
//    private val max_angle_Values = FloatArray(7) { 360.0f }
//    private val min_angle_Values = FloatArray(7) { -360.0f }

    private val max_Euler_angle_Values = floatArrayOf(180.0f, 180.0f, 180.0f)
    private val min_Euler_angle_Values = floatArrayOf(-180.0f, -180.0f, -180.0f)
    private val step = 0.5

    private val joint_cur_angle = FloatArray(7) { 0.0f }
    private val joint_stat = IntArray(7) { 0 }
    private val joint_isHome = IntArray(7) { 0 }
    private var Rob_stat:Int = 0
    private val joint_tar_angle_ModePos = FloatArray(7) { 0.0f }
    private val joint_tar_angle_ModeAngle = FloatArray(7) { 0.0f }
    private val Euler_tar_angle_ModePos = FloatArray(3) { 0.0f } //ID0 = EZ, ID1 = EY, ID2 = EX
    private val Euler_tar_XYZ_ModePos = FloatArray(3) { 0.0f } //ID0 = X, ID1 = Y, ID2 = Z
    private val joint_tar_angle = FloatArray(7) { 0.0f }
    private val joint_tar_speed = FloatArray(7) { 0.0f }
    private var joint_tar_accel = FloatArray(7) { 0.0f }
    private val Debug_Tag = "MainAct"


    private var JogSpeed: Float = 50.0f //unit: percent
    private var JogAccel: Float = 10.0f //unit: percent

    private var isEnablePressed = false
    private var isRobEnable = false
    private var isEStopActive = false
    private var lastToastTime = 0L
    private var lastSendTime = 0L

    //Default options
    private var currentJogModeByte: Byte = JogMode.J123
    private var currentJogMode: String = "Joint 1, 2, 3"
    private var currentCoorSys: String = "Base"
    private var currentWobj: String = "Wobj0"

    //Default Joysticks value
    var joystickX: Float = 0.0f
    var joystickY: Float = 0.0f
    var joystickZ: Float = 0.0f

    var x_total_inc_mm = 0f
    var y_total_inc_mm = 0f
    var z_total_inc_mm = 0f

    var j1_total_inc = 0f
    var j2_total_inc = 0f
    var j3_total_inc = 0f
    var j4_total_inc = 0f
    var j5_total_inc = 0f
    var j6_total_inc = 0f
    var EX_inc : Float = 0.0f
    var EY_inc : Float = 0.0f
    var EZ_inc : Float = 0.0f

    var TCP_Cur_X_mm: Float = 0.0f
    var TCP_Cur_Y_mm: Float = 0.0f
    var TCP_Cur_Z_mm: Float = 0.0f
    var EX_Cur: Float = 0.0f
    var EY_Cur: Float = 0.0f
    var EZ_Cur: Float = 0.0f

    var TCP_Tar_X_mm: Float = 0.0f
    var TCP_Tar_Y_mm: Float = 0.0f
    var TCP_Tar_Z_mm: Float = 0.0f
    var EX_Tar: Float = 0.0f
    var EY_Tar: Float = 0.0f
    var EZ_Tar: Float = 0.0f

    // ------------------ Standard D-H parameters (mm) ------------------
    private val PI_OVER_2: Float = (PI / 2.0).toFloat()
    private val NEG_PI_OVER_2: Float = (-PI / 2.0).toFloat()
    private val PI_FLOAT: Float = PI.toFloat()
    private val TWO_PI_FLOAT: Float = (2.0 * PI).toFloat()

    var A1: Float = 55f
    var D1: Float = 321.7f
    var ALPHA1: Float = PI_OVER_2
    var THETA1_OFFSET: Float = 0f

    var A2: Float = 330f
    var D2: Float = 0f
    var ALPHA2: Float = 0f
    var THETA2_OFFSET: Float = PI_OVER_2

    var A3: Float = 73f
    var D3: Float = 0f
    var ALPHA3: Float = PI_OVER_2
    var THETA3_OFFSET: Float = 0f

    var A4: Float = 0f
    var D4: Float = 330f
    var ALPHA4: Float = PI_OVER_2
    var THETA4_OFFSET: Float = 0f

    var A5: Float = 0f
    var D5: Float = 0f
    var ALPHA5: Float = NEG_PI_OVER_2
    var THETA5_OFFSET: Float = 0f

    var A6: Float = 0f
    var D6: Float = 175.5f
    var ALPHA6: Float = 0f
    var THETA6_OFFSET: Float = 0f

    var FLANGE_A: Float = 0f
    var FLANGE_D: Float = 107f
    var FLANGE_ALPHA: Float = 0f
    var FLANGE_THETA: Float = PI_FLOAT

    // Base offset (mm)
    var BASE_X_OFFSET: Float = 0.0f

    var dhParams = listOf(
        DHParam(
            a = A1,
            d = D1,
            alpha = ALPHA1,
            thetaOffset = THETA1_OFFSET
        ),
        DHParam(
            a = A2,
            d = D2,
            alpha = ALPHA2,
            thetaOffset = THETA2_OFFSET
        ),
        DHParam(
            a = A3,
            d = D3,
            alpha = ALPHA3,
            thetaOffset = THETA3_OFFSET
        ),
        DHParam(
            a = A4,
            d = D4,
            alpha = ALPHA4,
            thetaOffset = THETA4_OFFSET
        ),
        DHParam(
            a = A5,
            d = D5,
            alpha = ALPHA5,
            thetaOffset = THETA5_OFFSET
        ),
        DHParam(
            a = A6,
            d = D6,
            alpha = ALPHA6,
            thetaOffset = THETA6_OFFSET
        )
    )

    var flangeParam = DHParam(
        a = FLANGE_A,
        d = FLANGE_D,
        alpha = FLANGE_ALPHA,
        thetaOffset = FLANGE_THETA
    )

    var model = RobotModel(
        dhParams = dhParams,
        flange = flangeParam,
        baseXOffset = BASE_X_OFFSET,
        jointDirection = floatArrayOf(0f, 1f, -1f, -1f, 1f, 1f, 1f)
    )
    //unit: pulse per second
    val Vstep = 10000f
    val Vezi = 80000f
    //Pulse per round without gearbox
    val step_ppr = 200f
    val ezi_ppr = 10000f
    val AccStep = Vstep/1.2f        //unit: step/sec^2
    val tAccStep= Vstep/AccStep     //unit: sec
    var tAccCom = tAccStep          //Choosing accel time of step as common because step is much
                                    //slower than Ezi servo
    val gearRatio = floatArrayOf(0f, 0.004375f, 0.00757f, 0.02118f, 0.05901f, 0.0024f, 0.0125f)
    data class Joint(val id: Int, var Angle: Float, var Speed: Float, var stat: Byte)
    data class JointsData(val joints: MutableList<Joint>)

    val gson = Gson()
    val fk = RobotFK(model)
    val ik = RobotIK(model, fk, min_angle_Values, max_angle_Values, false)
    val L_planner = LinearPlanner(ik, fk, true)
    val Orient_planner = LinearPlanner(ik, fk, false)
    var json_send_String: String? = null
    var json_receive_String: String? = null
    private var jogJob: Job? = null


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
        setContentView(R.layout.activity_main)
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }
        RequestNeededPermissions()
        lifecycleScope.launch {
            delay(1000)
            uiReady = true
        }
        client = ESP32_Server(esp32_Ip, esp32_Port,this,false)
        lifecycleScope.launch(Dispatchers.IO) {
            isConnected = client.connect()
            if (!isConnected) {
                return@launch
            }
            while (isActive) {
                val line = client.receive()  // <= CHỈ DÙNG receive()
                if (!line.isNullOrEmpty()) {
                    val feedback = Esp32Parser.parse(line)
                    if (feedback != null) {
                        handleJsonFeedback(feedback,false)
                        if (isRobError == false) {
                            CalFK()
                            withContext(Dispatchers.Main){
                                txtView_X_cur.text = " ${TCP_Cur_X_mm} mm"
                                txtView_Y_cur.text = " ${TCP_Cur_Y_mm} mm"
                                txtView_Z_cur.text = " ${TCP_Cur_Z_mm} mm"

                                txtView_EZ_cur.text = "${EZ_Cur}°"
                                txtView_EY_cur.text = "${EY_Cur}°"
                                txtView_EX_cur.text = "${EX_Cur}°"
                            }
                        }
//                        Log.i(Debug_Tag,"Still get feedback")
                    }
                }
                delay(5)
            }
        }
        initView()      //Init view
        lifecycleScope.launch(Dispatchers.Main) {
            while (isActive) {
                if (uiReady && hasNewData) {
                    updateUIAngles(joint_cur_angle, false)
                    updateJointHomeIcon(imageView_J1_HomeFlag,joint_stat[1],joint_isHome[1],false)
                    updateJointHomeIcon(imageView_J2_HomeFlag,joint_stat[2],joint_isHome[2],false)
                    updateJointHomeIcon(imageView_J3_HomeFlag,joint_stat[3],joint_isHome[3],false)
                    updateJointHomeIcon(imageView_J4_HomeFlag,joint_stat[4],joint_isHome[4],false)
                    updateJointHomeIcon(imageView_J5_HomeFlag,joint_stat[5],joint_isHome[5],false)
                    updateJointHomeIcon(imageView_J6_HomeFlag,joint_stat[6],joint_isHome[6],false)
                    hasNewData = false
                }
                delay(50) //
            }
        }
        lifecycleScope.launch(Dispatchers.Main) {
            while (isActive) {
                if (lastConnectionState != isConnected) {
                    lastConnectionState = isConnected

                    if (!isConnected) {
                        showDialog("Robot not connected","ESP32_Server class")
                    }
                }
                delay(500)
            }
        }

        /*
        //Copy json into the internal memory of the tablet
        copyJsonToInternalStorage(this,"Joints_send.json")
        copyJsonToInternalStorage(this,"Joints_feedback.json")
        json_send_String = readJson(this,"Joints_send.json")
        Log.i("JSON_READ", "json_send_String: ${json_send_String}")
        json_receive_String = readJson(this,"Joints_feedback.json")
        Log.i("JSON_READ", "json_receive_String: ${json_receive_String}")*/


        //------------Begin of Jog mode------------
        checkCoorSys()
        checkUIModeJog()
        checkWobj()
        //------------=End of Jog mode-=-----------
        checkSpeedJog()     // Check speedBar trc khi jog Robot
        Ena_btn()           // Check nut enable trc khi jog Robot
        Estop_btn()         // Check nut Estop
        Home_btn()          // Check nut Home

        move_btn_Posi()
        Calib_btn()         // Check nut Calib
        solve_btn()
        MovelinZ_btn()
        MovelinY_btn()
        MovelinX_btn()
        btn_Xp()
        btn_Yp()
        btn_Zp()
        btn_Xm()
        btn_Ym()
        btn_Zm()
        MoveOriZ_btn()
        MoveOriY_btn()
        MoveOriX_btn()
        joystickXY()        // Joystick 1
        joystickZ()         // Joystick 2

//        testOri()
//        testFK()
//        testInterpolate()
//        testPlanner()
//        testIK()
//        testMoveLinearZ()
    }

    private fun Interpolate(curr_joint:FloatArray,tar_joint:FloatArray,verbose: Boolean = false){
        require(curr_joint.size == 7) { "curr_joint must have size = 7" }
        require(tar_joint.size == 7) { "tar_joint must have size = 7" }
        var delta_arr =  FloatArray(7) { 0.0f }
        var Ti_arr =  FloatArray(7) { 0.0f }

        for(i in 1..6){
            delta_arr[i] = abs(curr_joint[i] - tar_joint[i]) //unit: degree
            if (i==1||i==5||i==6) {
                                    //Convert from degree to pulse
                Ti_arr[i] = ((step_ppr/gearRatio[i])/360f * delta_arr[i]) / Vstep
            }
                                    //Convert from degree to pulse
            else Ti_arr[i] = ((ezi_ppr/gearRatio[i])/360f * delta_arr[i]) / Vezi
        }

        val index = indexOfSlowestJoint(Ti_arr)
        if (verbose){
            Log.i("Interpolate", "Delta = ${delta_arr.contentToString()}")
            Log.i("Interpolate", "Time = ${Ti_arr.contentToString()}, T slowest at $index")
        }
        var Velo_arr =  FloatArray(7) { 0.0f }
        val T_total = Ti_arr[index] + 2*tAccCom

        for(i in 1..6){
            if (i==1||i==5||i==6) {
                Velo_arr[i] = ((step_ppr/gearRatio[i])/360f * delta_arr[i]) / T_total
            }
            else Velo_arr[i] = ((ezi_ppr/gearRatio[i])/360f * delta_arr[i]) / T_total

        }
        clampVelocityToLimit(Velo_arr,Vstep,Vezi)
        convertVelocityToPercent(Velo_arr,Vstep,Vezi)
        for(i in 1..6){
            joint_tar_speed[i] = Velo_arr[i]
        }
        joint_tar_accel = FloatArray(7) { tAccCom }
        if (verbose){
            Log.i("Interpolate", "Accel time = ${tAccCom}s")
            Log.i("Interpolate", "Velocity = ${joint_tar_speed.contentToString()}")
        }
    }
    private fun indexOfSlowestJoint(arr: FloatArray): Int {
        require(arr.size == 7) { "Array size must be exactly 7" }
        var maxIndex = 0
        var maxVal = arr[0]
        for (i in 1..6) {
            if (arr[i] > maxVal) {
                maxVal = arr[i]
                maxIndex = i
            }
        }
        return maxIndex
    }
    private fun clampVelocityToLimit(veloArr: FloatArray, Vstep: Float, Vezi: Float) {
        require(veloArr.size == 7) { "Velocity array must have size 7" }

        for (i in 1..6) {
            val limit = if (i == 1 || i == 5 || i == 6) Vstep else Vezi
            if (veloArr[i] > limit) {
                veloArr[i] = limit
            }
        }
    }
    private fun convertVelocityToPercent(veloArr: FloatArray, Vstep: Float, Vezi: Float) {
        require(veloArr.size == 7) { "Velocity array must have size 7" }

        for (i in 1..6) {
            val maxSpeed = if (i == 1 || i == 5 || i == 6) Vstep else Vezi
            val ratio = veloArr[i] / maxSpeed
            val clampedRatio = ratio.coerceIn(-1f, 1f)
            veloArr[i] = clampedRatio * 100f
        }
    }
    private fun degPerSecToPulse(veloDeg: FloatArray, Vstep: Float, Vezi: Float): FloatArray {
        val stepPPR = 200f
        val eziPPR = 10000f
        val veloPulse = FloatArray(7)
        for (i in 1..6) {
            val ppr = if (i == 1 || i == 5 || i == 6) stepPPR else eziPPR
            veloPulse[i] =
                (ppr / gearRatio[i]) * veloDeg[i] / 360f
        }
        return veloPulse
    }
    private fun InterpolateAndSend(
        jointPath: List<FloatArray>,
        dt: Float,
        Vstep: Float,
        Vezi: Float,
        tAccCom: Float,
        mode: Byte,
        enable: Boolean,
        verbose: Boolean = true
    ) {
        if (jointPath.size < 2) {
            if (verbose) {
                Log.w("InterpolateAndSend", "jointPath size < 2, nothing to interpolate")
                showDialog("No solution for this move","Interpolate")
            }
            return
        }
        lifecycleScope.launch(Dispatchers.IO) {
//            require(jointPath.size >= 2) {
//                "Need at least 2 joint points"
//            }
            for (k in 0 until jointPath.size - 1) {

                val qCurr = jointPath[k]
                val qNext = jointPath[k + 1]

                val veloDeg = FloatArray(7)
                var dqMax = 0f
                for (i in 1..6) {
                    val dq = abs(qNext[i] - qCurr[i])
                    if (dq > dqMax) dqMax = dq
                }

                if (dqMax < 1e-6f) continue

                val vMax = dqMax / dt

                for (i in 1..6) {
                    val dq = abs(qNext[i] - qCurr[i])
                    veloDeg[i] = vMax * dq / dqMax
                }

                val veloPulse = degPerSecToPulse(veloDeg, Vstep, Vezi)
                clampVelocityToLimit(veloPulse, Vstep, Vezi)
                convertVelocityToPercent(veloPulse, Vstep, Vezi)

                val accelArr = FloatArray(7) { tAccCom }

                val json = buildJogJoystickPacket(
                    FlagCommand = CommandFlags.Move,
                    mode = mode,
                    enable = enable,
                    jointsAngle = qNext,
                    jointsSpeed = veloPulse,
                    jointsAccel = accelArr,
                    false
                )
                client.send(json)
                delay((dt * 1000).toLong())
            }
        }
    }

    private fun handleJsonFeedback(f: Esp32Feedback, verbose:Boolean = false) {
        f.current.angle.forEachIndexed { index, value ->
            joint_cur_angle[index + 1] = value
        }
        f.current.stat.forEachIndexed { index, value ->
            joint_stat[index + 1] = value
        }
        f.current.home.forEachIndexed { index, value ->
            joint_isHome[index + 1] = value
        }
        isRobEnable = (f.enable==1)
        Rob_stat = f.rob_status

        if (Rob_stat != lastRobStat) {
            lastRobStat = Rob_stat
            when (Rob_stat) {
                StatusFlags.LIM.toInt() -> {
                    isRobError = true
                    if(verbose) Log.i(Debug_Tag,"Lost Calibration")
                    showErrorOnce("Lost Calibration")
                }
                StatusFlags.LIE.toInt() -> {
                    isRobError = true
                    if(verbose) Log.i(Debug_Tag,"Lost Encoder")
                    showErrorOnce("Lost Encoder")
                }
                StatusFlags.LI.toInt()  -> {
                    isRobError = true
                    if(verbose) Log.i(Debug_Tag,"Lost I2C")
                    showErrorOnce("Lost I2C")
                }
                StatusFlags.Error.toInt() ->{
                    isRobError = true
                    if(verbose) Log.i(Debug_Tag,"Error")
                    showErrorOnce("Error")
                }

                StatusFlags.OOS.toInt() ->{
                    isRobError = true
                    if(verbose) Log.i(Debug_Tag,"OOS")
                    showErrorOnce("OOS")
                }

                StatusFlags.OOR.toInt() ->{
                    isRobError = true
                    if(verbose) Log.i(Debug_Tag,"OOR")
                    showErrorOnce("OOR")
                }
                StatusFlags.Calibrated.toInt() ->{
                    isRobError = false
                    if(verbose) Log.i(Debug_Tag,"Calibrated")
                    showErrorOnce("Robot is Calibrated")
                }

                else -> isRobError = false
            }
        }


        hasNewData = true
        if(verbose){
            Log.i(Debug_Tag, "Rob_Status: ${Rob_stat}")
            Log.i(Debug_Tag, "Angles: ${joint_cur_angle.joinToString()}")
            Log.i(Debug_Tag, "Enable: ${isRobEnable}")
            Log.i(Debug_Tag, "Joint Status: ${joint_stat.joinToString()}")
            Log.i(Debug_Tag, "Joint isHome: ${joint_isHome.joinToString()}")
            Log.i(Debug_Tag, " ")
        }
    }
    private fun initView(){
        joystick1 = findViewById(R.id.joyStick1)
        joystick2 = findViewById(R.id.joyStick2)
        btnEnable = findViewById(R.id.btn_Ena)
        btnEstop = findViewById(R.id.btn_Stop)
        btnMove = findViewById(R.id.btn_Move)
        btnSolve = findViewById(R.id.btn_Solve)
        btnEstop = findViewById(R.id.btn_Stop)
        btnHome = findViewById(R.id.btn_Home)
        btnCalib = findViewById(R.id.btn_calib)
        btnMoveLinearZ = findViewById(R.id.btn_moveLinZ)
        btnMoveLinearY = findViewById(R.id.btn_moveLinY)
        btnMoveLinearX = findViewById(R.id.btn_moveLinX)
        btnMoveOrientZ = findViewById(R.id.btn_moveOriZ)
        btnMoveOrientY = findViewById(R.id.btn_moveOriY)
        btnMoveOrientX = findViewById(R.id.btn_moveOriX)

        btn_Zp = findViewById(R.id.btn_Zp)
        btn_Zm = findViewById(R.id.btn_Zm)
        btn_Yp = findViewById(R.id.btn_Yp)
        btn_Ym = findViewById(R.id.btn_Ym)
        btn_Xp = findViewById(R.id.btn_Xp)
        btn_Xm = findViewById(R.id.btn_Xm)

        spinnerJogMode = findViewById(R.id.spinnerJogMode)
        imageView_J1_HomeFlag = findViewById(R.id.imageView_J1_HomeFlag)
        imageView_J2_HomeFlag = findViewById(R.id.imageView_J2_HomeFlag)
        imageView_J3_HomeFlag = findViewById(R.id.imageView_J3_HomeFlag)
        imageView_J4_HomeFlag = findViewById(R.id.imageView_J4_HomeFlag)
        imageView_J5_HomeFlag = findViewById(R.id.imageView_J5_HomeFlag)
        imageView_J6_HomeFlag = findViewById(R.id.imageView_J6_HomeFlag)


        accelBar = findViewById(R.id.accelBar)
        accelView = findViewById(R.id.accelView)
        accelBar.progress = 20
        val accelViewFormatted = String.format("%.1f", JogAccel.toFloat())
        accelView.text = "${accelViewFormatted}%"

        speedBar = findViewById(R.id.speedBar)
        speedView = findViewById(R.id.speedView)
        speedBar.progress = 100
        val speedViewFormatted = String.format("%.1f", JogSpeed.toFloat())
        speedView.text = "${speedViewFormatted}%"

        //--------------------------Angle--------------------------------
        txtView_J1_cur_angle = findViewById(R.id.txtView_J1_curAngle)
        txtView_J2_cur_angle = findViewById(R.id.txtView_J2_curAngle)
        txtView_J3_cur_angle = findViewById(R.id.txtView_J3_curAngle)
        txtView_J4_cur_angle = findViewById(R.id.txtView_J4_curAngle)
        txtView_J5_cur_angle = findViewById(R.id.txtView_J5_curAngle)
        txtView_J6_cur_angle = findViewById(R.id.txtView_J6_curAngle)

        txtView_X_cur = findViewById(R.id.txtView_X_Cur)
        txtView_Y_cur = findViewById(R.id.txtView_Y_Cur)
        txtView_Z_cur = findViewById(R.id.txtView_Z_Cur)

        txtviewX = findViewById(R.id.txtviewX)
        txtviewY = findViewById(R.id.txtviewY)
        txtviewZ = findViewById(R.id.txtviewZ)

        txtviewEX = findViewById(R.id.txtviewEX)
        txtviewEY = findViewById(R.id.txtviewEY)
        txtviewEZ = findViewById(R.id.txtviewEZ)

        txtView_EX_cur = findViewById(R.id.txtView_EX_Cur)
        txtView_EY_cur = findViewById(R.id.txtView_EY_Cur)
        txtView_EZ_cur = findViewById(R.id.txtView_EZ_Cur)


        txtviewJ1 = findViewById(R.id.txtviewJ1)
        txtviewJ2 = findViewById(R.id.txtviewJ2)
        txtviewJ3 = findViewById(R.id.txtviewJ3)
        txtviewJ4 = findViewById(R.id.txtviewJ4)
        txtviewJ5 = findViewById(R.id.txtviewJ5)
        txtviewJ6 = findViewById(R.id.txtviewJ6)

        editTxt_X_tar = findViewById(R.id.editTxt_tar_X)
        editTxt_Y_tar = findViewById(R.id.editTxt_tar_Y)
        editTxt_Z_tar = findViewById(R.id.editTxt_tar_Z)

        sBarJ1_angle = findViewById(R.id.sBarJ1_angle)
        txtViewJ1_tarAngle = findViewById(R.id.txtViewJ1_tarAngle)
        sBarJ2_angle = findViewById(R.id.sBarJ2_angle)
        txtViewJ2_tarAngle = findViewById(R.id.txtViewJ2_tarAngle)
        sBarJ3_angle = findViewById(R.id.sBarJ3_angle)
        txtViewJ3_tarAngle = findViewById(R.id.txtViewJ3_tarAngle)
        sBarJ4_angle = findViewById(R.id.sBarJ4_angle)
        txtViewJ4_tarAngle = findViewById(R.id.txtViewJ4_tarAngle)
        sBarJ5_angle = findViewById(R.id.sBarJ5_angle)
        txtViewJ5_tarAngle = findViewById(R.id.txtViewJ5_tarAngle)
        sBarJ6_angle = findViewById(R.id.sBarJ6_angle)
        txtViewJ6_tarAngle = findViewById(R.id.txtViewJ6_tarAngle)

        sBarEZ_angle = findViewById(R.id.sBarEZ_angle)
        txtViewEZ_tarAngle = findViewById(R.id.txtViewEZ_tarAngle)
        sBarEY_angle = findViewById(R.id.sBarEY_angle)
        txtViewEY_tarAngle = findViewById(R.id.txtViewEY_tarAngle)
        sBarEX_angle = findViewById(R.id.sBarEX_angle)
        txtViewEX_tarAngle = findViewById(R.id.txtViewEX_tarAngle)

        hideJogByAngle()
        hideCalib_view()

        setupSBar_angle(sBarJ1_angle, txtViewJ1_tarAngle,1)
        setupSBar_angle(sBarJ2_angle, txtViewJ2_tarAngle,2)
        setupSBar_angle(sBarJ3_angle, txtViewJ3_tarAngle,3)
        setupSBar_angle(sBarJ4_angle, txtViewJ4_tarAngle,4)
        setupSBar_angle(sBarJ5_angle, txtViewJ5_tarAngle,5)
        setupSBar_angle(sBarJ6_angle, txtViewJ6_tarAngle,6)

        setupSBar_Euler_angle(sBarEZ_angle,txtViewEZ_tarAngle,0)
        setupSBar_Euler_angle(sBarEY_angle,txtViewEY_tarAngle,1)
        setupSBar_Euler_angle(sBarEX_angle,txtViewEX_tarAngle,2)
        setupedit_TxtView(editTxt_X_tar)
        setupedit_TxtView(editTxt_Y_tar)
        setupedit_TxtView(editTxt_Z_tar)
    }
    private fun roundIfTiny(v: Float) = if (abs(v) < 1e-3f) 0f else (round(v * 1000) / 1000.0f)

    //Begin of View related function
    fun hideJogByAngle() {
        val jogAngleViews = listOf(
            accelView,accelBar,
            sBarJ1_angle, txtViewJ1_tarAngle,
            sBarJ2_angle, txtViewJ2_tarAngle,
            sBarJ3_angle, txtViewJ3_tarAngle,
            sBarJ4_angle, txtViewJ4_tarAngle,
            sBarJ5_angle, txtViewJ5_tarAngle,
            sBarJ6_angle, txtViewJ6_tarAngle,
            txtviewJ1,txtviewJ2,txtviewJ3,txtviewJ4,txtviewJ5,txtviewJ6
        )
        jogAngleViews.forEach { it.visibility = View.GONE }
    }
    fun showJogByAngle() {
        val jogAngleViews = listOf(
            accelView,accelBar,
            sBarJ1_angle, txtViewJ1_tarAngle,
            sBarJ2_angle, txtViewJ2_tarAngle,
            sBarJ3_angle, txtViewJ3_tarAngle,
            sBarJ4_angle, txtViewJ4_tarAngle,
            sBarJ5_angle, txtViewJ5_tarAngle,
            sBarJ6_angle, txtViewJ6_tarAngle,
            txtView_J1_cur_angle, txtView_J2_cur_angle,
            txtView_J3_cur_angle, txtView_J4_cur_angle,
            txtView_J5_cur_angle, txtView_J6_cur_angle,
            txtviewJ1,txtviewJ2,txtviewJ3,txtviewJ4,txtviewJ5,txtviewJ6
        )
        jogAngleViews.forEach { it.visibility = View.VISIBLE }
    }

    fun hideCalib_view() {
        val calibView = listOf(
            accelView,accelBar,btnCalib,
            btnMoveLinearZ,btnMoveLinearY,btnMoveLinearX,
            btnMoveOrientZ,btnMoveOrientY,btnMoveOrientX
        )
        calibView.forEach { it.visibility = View.GONE }
    }
    fun showJCalib_view() {
        val calibView = listOf(
            accelView,accelBar,btnCalib,
            btnMoveLinearZ,btnMoveLinearY,btnMoveLinearX,
            btnMoveOrientZ,btnMoveOrientY,btnMoveOrientX
        )
        calibView.forEach { it.visibility = View.VISIBLE }
    }

    fun showJoystick(){
        val view_Joystick = listOf(
            joystick1,joystick2
        )
        view_Joystick.forEach { it.visibility = View.VISIBLE }
    }
    fun hideJoystick(){
        val view_Joystick = listOf(
            joystick1,joystick2
        )
        view_Joystick.forEach { it.visibility = View.GONE }
    }

    fun showButton(){
        val view_Joystick = listOf(
            btn_Zp,btn_Zm,btn_Yp,btn_Ym,btn_Xp,btn_Xm
        )
        view_Joystick.forEach { it.visibility = View.VISIBLE }
    }
    fun hideButton(){
        val view_Joystick = listOf(
            btn_Zp,btn_Zm,btn_Yp,btn_Ym,btn_Xp,btn_Xm
        )
        view_Joystick.forEach { it.visibility = View.GONE }
    }

    fun hideJogbyPos(){
        val jogPosViews = listOf(
            accelView,accelBar,
            txtviewX,txtviewY,txtviewZ,
            txtviewEX,txtviewEY,txtviewEZ,
            sBarEZ_angle,sBarEY_angle,sBarEX_angle,
            txtViewEZ_tarAngle,txtViewEY_tarAngle,txtViewEX_tarAngle,
            btnMove,btnSolve,
            editTxt_X_tar,editTxt_Y_tar,editTxt_Z_tar
        )
        jogPosViews.forEach { it.visibility = View.GONE }


    }
    fun showJogbyPos(){
        val jogPosViews = listOf(
            accelView,accelBar,
            txtviewX,txtviewY,txtviewZ,
            txtviewEX,txtviewEY,txtviewEZ,
            sBarEZ_angle,sBarEY_angle,sBarEX_angle,
            txtViewEZ_tarAngle,txtViewEY_tarAngle,txtViewEX_tarAngle,
            btnMove,btnSolve,
            editTxt_X_tar,editTxt_Y_tar,editTxt_Z_tar
        )
        jogPosViews.forEach { it.visibility = View.VISIBLE }

    }
    //End of View related function

    private fun CalFK(verbose: Boolean = false){
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val pos = fk.getTCPPosition(T)
        TCP_Cur_X_mm = roundIfTiny(pos[0])
        TCP_Cur_Y_mm = roundIfTiny(pos[1])
        TCP_Cur_Z_mm = roundIfTiny(pos[2])
        val orient = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(orient)
        EZ_Cur = roundIfTiny(euler[0])
        EY_Cur = roundIfTiny(euler[1])
        EX_Cur = roundIfTiny(euler[2])
        if (verbose){
            Log.i("Cal FK"," TCP_Cur_X_mm = $TCP_Cur_X_mm")
            Log.i("Cal FK"," TCP_Cur_Y_mm = $TCP_Cur_Y_mm")
            Log.i("Cal FK"," TCP_Cur_Z_mm = $TCP_Cur_Z_mm")
            Log.i("Cal FK"," EZ_Cur = $EZ_Cur")
            Log.i("Cal FK"," EY_Cur = $EY_Cur")
            Log.i("Cal FK"," EX_Cur = $EX_Cur")
        }
    }

    //Begin of setup jogByAngle
    private fun setupSBar_angle(seekBar: SeekBar, txtView: TextView, jointID: Int) {
        val ANGLE_SCALE = 2

        /*
        val min = min_angle_Values[jointID]
        val max = max_angle_Values[jointID]
        val totalSteps = ((max - min) / step).toInt()
        seekBar.max = totalSteps

        val initialProgress = ((0.0 - min) / step).roundToInt().coerceIn(0, totalSteps)
        seekBar.progress = initialProgress
        update_txtView_angle(txtView, initialProgress, jointID)

        seekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                if (currentJogMode != "Angle") return
                update_txtView_angle(txtView, progress, jointID)
            }

            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })*/

        val minInt = (min_angle_Values[jointID] * ANGLE_SCALE).toInt()
        val maxInt = (max_angle_Values[jointID] * ANGLE_SCALE).toInt()
        seekBar.max = maxInt - minInt
        seekBar.progress = -minInt
        update_txtView_angle(txtView, seekBar.progress, jointID)
        seekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(
                seekBar: SeekBar?,
                progress: Int,
                fromUser: Boolean
            ) {
                if (currentJogMode != "Angle") return
                update_txtView_angle(txtView, progress, jointID)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })
    }
    private fun update_txtView_angle(txtView: TextView, progress: Int, jointID: Int) {
        /*
        val min = min_angle_Values[jointID]
        val value = (min + progress * step).toFloat()
        val formatted = String.format("%.1f°", value)
        txtView.text = formatted

        joint_tar_angle_ModeAngle[jointID] = value*/
        val ANGLE_SCALE = 2
        val minInt = (min_angle_Values[jointID] * ANGLE_SCALE).toInt()

        val value = (minInt + progress).toFloat() / ANGLE_SCALE

        txtView.text = String.format("%.1f°", value)
        joint_tar_angle_ModeAngle[jointID] = value
    }

    // ID0 = EZ, ID1 = EY, ID2 = EX
    private fun setupSBar_Euler_angle(seekBar: SeekBar, txtView: TextView, ID: Int, verbose: Boolean = false) {
        val STEP_INT = 1
        val SCALE = 2
        val min = (min_Euler_angle_Values[ID] * SCALE).toInt()
        val max = (max_Euler_angle_Values[ID] * SCALE).toInt()
        seekBar.max = max - min
        seekBar.progress = -min

        update_txtView_Euler_angle(txtView, seekBar.progress, ID)

        seekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                if (currentJogMode != "Position") return
                update_txtView_Euler_angle(txtView, progress, ID)
                isTarChanged = true
            }

            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })
    }
    private fun update_txtView_Euler_angle(txtView: TextView, progress: Int ,ID: Int,verbose: Boolean = false) {
        val STEP_INT = 1
        val SCALE = 2
        val minInt = (min_Euler_angle_Values[ID] * SCALE).toInt()
        val value = (minInt + progress).toFloat() / SCALE

        txtView.text = String.format("%.1f°", value)
        Euler_tar_angle_ModePos[ID] = value

        if (verbose) Log.i("Euler", "ID=$ID value=$value")

        /*
        val min = min_Euler_angle_Values[ID]
        val value = (min + progress * step).toFloat()
        val formatted = String.format("%.1f°", value)
        txtView.text = formatted
        if (verbose) Log.i("update_txtView_Euler_angle","Check $ID: $value ")
        Euler_tar_angle_ModePos[ID] = value*/
    }

    private fun setupedit_TxtView(editTxt: EditText){
        editTxt.addTextChangedListener(object : TextWatcher {
            override fun beforeTextChanged(
                s: CharSequence?, start: Int, count: Int, after: Int
            ) {}
            override fun onTextChanged(
                s: CharSequence?, start: Int, before: Int, count: Int
            ) {
                isTarChanged = true
            }
            override fun afterTextChanged(s: Editable?) {
                isTarChanged = true
            }
        })
    }
    private fun update_tar_coordinate(verbose: Boolean = true): Boolean{
        val x = editTxt_X_tar.text.toString().toFloatOrNull()
        val y = editTxt_Y_tar.text.toString().toFloatOrNull()
        val z = editTxt_Z_tar.text.toString().toFloatOrNull()
        if (verbose) Log.i("update_tar_coordinate"," x= $x, y= $y, z= $z")
        if (x == null || y == null || z == null) return false
        else{
            Euler_tar_XYZ_ModePos[0] = x
            Euler_tar_XYZ_ModePos[1] = y
            Euler_tar_XYZ_ModePos[2] = z
            return true
        }
    }
    //End of setup jogByAngle

    //Begin of check function
    private fun checkUIModeJog(){
        val directionContainer = findViewById<FrameLayout>(R.id.jogContainer)
        setupSpinner(
            context = this,
            spinner = findViewById(R.id.spinnerJogMode),
            arrayRes = R.array.JogMode,
            layoutRes = R.layout.spinner_layout
        ) { selected ->
            currentJogMode = selected
            when (selected) {
                "Linear" -> {
                    if (isRobError == true){
                        showDialog("Cannot run Linear mode due to lost calibration","Robot hardware")
                    }
                    else{
                        Toast.makeText(this@MainActivity, "Linear mode active", Toast.LENGTH_SHORT).show()
                    }
                    currentJogModeByte = JogMode.Linear
                    showButton()
                    hideJoystick()
                    directionContainer.setBackgroundResource(R.drawable.show_dir_xyz)
                    hideJogByAngle()
                    hideCalib_view()
                    hideJogbyPos()
                }
                "Reorient" -> {
                    currentJogModeByte = JogMode.Reori
                    if (isRobError == true){
                        showDialog("Cannot run Reorient mode due to lost calibration","Robot hardware")
                    }
                    else{
                        Toast.makeText(this@MainActivity, "Reorient mode active", Toast.LENGTH_SHORT).show()
                    }
                    showJoystick()
                    directionContainer.setBackgroundResource(R.drawable.show_dir_xyz)
                    hideJogByAngle()
                    hideCalib_view()
                    hideJogbyPos()
                    hideButton()
                }
                "Joint 1, 2, 3" -> {
                    currentJogModeByte = JogMode.J123
                    Toast.makeText(this@MainActivity, "Joint 1, 2, 3 mode active", Toast.LENGTH_SHORT).show()
                    showJoystick()
                    directionContainer.setBackgroundResource(R.drawable.show_dir_123)
                    hideJogByAngle()
                    hideCalib_view()
                    hideJogbyPos()
                    hideButton()
                }
                "Joint 4, 5, 6" -> {
                    currentJogModeByte = JogMode.J456
                    Toast.makeText(this@MainActivity, "Joint 4, 5, 6 mode active", Toast.LENGTH_SHORT).show()
                    showJoystick()
                    directionContainer.setBackgroundResource(R.drawable.show_dir_456)
                    hideJogByAngle()
                    hideCalib_view()
                    hideJogbyPos()
                    hideButton()
                }
                "Angle" -> {
                    currentJogModeByte = JogMode.Angle
                    if (isRobError == true){
                        showDialog("Cannot run Angle mode due to lost calibration","Robot hardware")
                    }
                    else{
                        Toast.makeText(this@MainActivity, "Move by angle mode active", Toast.LENGTH_SHORT).show()
                    }
                    hideJogbyPos()
                    hideCalib_view()
                    showJogByAngle()
                    hideButton()
                    checkAccelJog()
                    hideJoystick()
                    directionContainer.setBackgroundResource(R.drawable.svg_acceleration)
                }
                "Calibrate" -> {
                    currentJogModeByte = JogMode.Calib
                    Toast.makeText(this@MainActivity, "Calibrate mode active", Toast.LENGTH_SHORT).show()
                    hideJogbyPos()
                    hideJoystick()
                    directionContainer.setBackgroundResource(R.drawable.svg_acceleration)
                    hideJogByAngle()
                    showJCalib_view()
                    hideButton()
                }
                "Position" -> {
                    currentJogModeByte = JogMode.Posi
                    if (isRobError == true){
                        showDialog("Cannot run Position mode due to lost calibration","Robot hardware")
                    }
                    else{
                        Toast.makeText(this@MainActivity, "Move to position mode active", Toast.LENGTH_SHORT).show()
                    }

                    hideJoystick()
                    hideCalib_view()
                    directionContainer.setBackgroundResource(R.drawable.svg_acceleration)
                    hideJogByAngle()
                    showJogbyPos()
                    hideButton()
                }
            }
            jogRobot(currentJogMode)
        }
    }
    private fun checkCoorSys(){
        setupSpinner(
            context = this,
            spinner = findViewById(R.id.spinnerCorSys),
            arrayRes = R.array.CoorSys,
            layoutRes = R.layout.spinner_layout
        ) { selected ->
            currentCoorSys = selected
        }
    }
    private fun checkWobj(){
        setupSpinner(
            context = this,
            spinner = findViewById(R.id.spinnerWobj),
            arrayRes = R.array.Wobj,
            layoutRes = R.layout.spinner_layout
        ) { selected ->
            currentWobj = selected
        }
    }
    private fun checkSpeedJog(){
        speedBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                JogSpeed = max(progress * 0.5f, 1f)
                speedView.text = "%.1f %% ".format(JogSpeed)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })
    }
    private fun checkAccelJog(){
        accelBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                JogAccel = max(progress * 0.5f, 1f)
                accelView.text = "%.1f %% ".format(JogAccel)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })
    }
    //End of check function

    //Begin of Jog function
    private fun jogRobot(mode: String) {
        var isJoystickMoving:Boolean
//        Log.i(Debug_Tag,"isJoystickMoving = $isJoystickMoving")
        jogJob?.cancel()
        jogJob = CoroutineScope(Dispatchers.Default).launch {
            while (isActive) {
                when (mode){
                    "Linear" -> {
                        isJoystickMoving = Joystick1Moving || Joystick2Moving
                        if(isJoystickMoving && isEnablePressed){
                            JogLinear()

                        }
                    }

                    "Reorient" -> {
                        isJoystickMoving = Joystick1Moving || Joystick2Moving
                        if(isJoystickMoving && isEnablePressed){
                            JogReorient()
//                            withContext(Dispatchers.Main){
//                                txtView_X_cur.text = " ${TCP_Cur_X_mm} mm"
//                                txtView_Y_cur.text = " ${TCP_Cur_Y_mm} mm"
//                                txtView_Z_cur.text = " ${TCP_Cur_Z_mm} mm"
//
//                                txtView_EZ_cur.text = "${EZ_Tar}°"
//                                txtView_EY_cur.text = "${EY_Tar}°"
//                                txtView_EX_cur.text = "${EX_Tar}°"
//                            }
                        }
                    }

                    "Joint 1, 2, 3" -> {
                        isJoystickMoving = Joystick1Moving || Joystick2Moving
//                        Log.i(Debug_Tag,"isJoystickMoving = $isJoystickMoving")
                        if(isJoystickMoving && isEnablePressed){
                            JogJ123()
                        }
                    }

                    "Joint 4, 5, 6" -> {
                        isJoystickMoving = Joystick1Moving || Joystick2Moving
//                        Log.i(Debug_Tag,"isJoystickMoving = $isJoystickMoving")
                        if(isJoystickMoving && isEnablePressed){
                            JogJ456()
                        }
                    }

                    "Calibrate" -> {
                        if(isRobError == false){

                        }
                    }

                    "Angle" -> {

                    }

                    "Position" -> {

                    }
                }
                delay(20)
            }
        }
    }
    private fun JogLinear(){
        if (isRobError == false){
            val T = fk.forwardKinematicsDeg(joint_cur_angle)
            val pos = fk.getTCPPosition(T)
            val orient = fk.getTCPOrientation(T)
            val euler = fk.getTCPEulerZYX(orient)

            EZ_Cur = roundIfTiny(euler[0])
            EY_Cur = roundIfTiny(euler[1])
            EX_Cur = roundIfTiny(euler[2])
            TCP_Cur_X_mm = roundIfTiny(pos[0])
            TCP_Cur_Y_mm = roundIfTiny(pos[1])
            TCP_Cur_Z_mm = roundIfTiny(pos[2])
//        Log.i("forwardKinematics","TCP_Cur = [${TCP_Cur_X_mm}," +
//                "${TCP_Cur_Y_mm}, ${TCP_Cur_Z_mm}]" )

            x_total_inc_mm += roundIfTiny((joystickX/100) * 2)
            y_total_inc_mm += roundIfTiny((joystickY/100) * 2)
            z_total_inc_mm += roundIfTiny((joystickZ/100) * 2)

            TCP_Tar_X_mm = roundIfTiny(x_total_inc_mm + TCP_Cur_X_mm)
            TCP_Tar_Y_mm = roundIfTiny(y_total_inc_mm + TCP_Cur_Y_mm)
            TCP_Tar_Z_mm = roundIfTiny(z_total_inc_mm + TCP_Cur_Z_mm)

            val T_inc = ik.eulerZYXToT(TCP_Tar_X_mm,TCP_Tar_Y_mm,TCP_Tar_Z_mm,EZ_Cur,EY_Cur,EX_Cur)
            val L_path = L_planner.incLinearByMM(T, T_inc,true,joint_cur_angle)
            if (L_path.isEmpty()) {
                val now = System.currentTimeMillis()
                if (now - lastToastTime > 2000) {
                    lastToastTime = now
                    runOnUiThread {
                        Toast.makeText(this, "Cannot reach!", Toast.LENGTH_LONG).show()
                        Log.e("JogLinear", "No IK solution for this JogLinear pose!")
                    }
                }
            }
            else{
                val debug:String = buildJogJoystickPacket(null,JogMode.Linear,
                    isEnablePressed,L_path, null, null,false)
//            Log.i("JogLinear", "Solution: ${L_path.joinToString { "%.3f°".format(it) }}")
//            Log.i("forwardKinematics","TCP_Tar = [${TCP_Tar_X_mm}," +
//                    "${TCP_Tar_Y_mm}, ${TCP_Tar_Z_mm}]" )
//            Log.i("Json String", "String: ${debug}")
                lifecycleScope.launch(Dispatchers.IO) {
                    client.send(debug)
//                Log.i("Json send ", "JogLinear String: ${debug}")
                }
            }
        }

    }
    private fun JogJ123(verbose:Boolean = true){
        val now = System.currentTimeMillis()
        if (now - lastSendTime < 200) return   // 200ms = 5Hz
        lastSendTime = now

        j1_total_inc = roundIfTiny((joystickX/100) * 2)
        joint_tar_angle[1] = j1_total_inc
        j2_total_inc = roundIfTiny((joystickY/100) * 2)
        joint_tar_angle[2] = j2_total_inc
        j3_total_inc = roundIfTiny((joystickZ/100) * 2)
        joint_tar_angle[3] = j3_total_inc


        val debug:String = buildJogJoystickPacket(CommandFlags.Move_Offset,JogMode.J123,
            isEnablePressed,joint_tar_angle,null,null,false)
        lifecycleScope.launch(Dispatchers.IO) {
            client.send(debug)
            if(verbose) Log.i("Json send ", "JogJ123 Offset String: ${debug}")
        }
    }
    private fun JogJ456(verbose:Boolean = true){
        val now = System.currentTimeMillis()
        if (now - lastSendTime < 200) return   // 200ms = 5Hz
        lastSendTime = now
        j4_total_inc = roundIfTiny((joystickX/100) * 2)
        joint_tar_angle[4] = j4_total_inc
        j5_total_inc = roundIfTiny((joystickY/100) * 2)
        joint_tar_angle[5] = j5_total_inc
        j6_total_inc = roundIfTiny((joystickZ/100) * 2)
        joint_tar_angle[6] = j6_total_inc
        val debug:String = buildJogJoystickPacket(CommandFlags.Move_Offset,JogMode.J456,
            isEnablePressed,joint_tar_angle,null,null,false)
        lifecycleScope.launch(Dispatchers.IO) {
            client.send(debug)
            if(verbose) Log.i("Json send ", "Jog456 Offset String: ${debug}")
        }
    }
    private fun JogReorient(){
        if (isRobError == false){
            if (((roundIfTiny(joint_cur_angle[5]) <= 0.1f)
                        && (roundIfTiny(joint_cur_angle[5]) >= -0.1f))){
                val now = System.currentTimeMillis()
                if (now - lastToastTime > 2000) {
                    lastToastTime = now
                    runOnUiThread {
                        Toast.makeText(this,
                            "Lost 1 DOF, please move Joint 5!",
                            Toast.LENGTH_LONG).show()
                    }
                }
                return
            }

            val T = fk.forwardKinematicsDeg(joint_cur_angle)
            val pos = fk.getTCPPosition(T)
            val orient = fk.getTCPOrientation(T)
            val euler = fk.getTCPEulerZYX(orient)

            TCP_Cur_X_mm = roundIfTiny(pos[0])
            TCP_Cur_Y_mm = roundIfTiny(pos[1])
            TCP_Cur_Z_mm = roundIfTiny(pos[2])
//        Log.i(Debug_Tag,"TCP_Cur_X_mm = $TCP_Cur_X_mm, TCP_Cur_Y_mm = $TCP_Cur_Y_mm, TCP_Cur_Z_mm = $TCP_Cur_Z_mm")
            EZ_Cur = roundIfTiny(euler[0])
            EY_Cur = roundIfTiny(euler[1])
            EX_Cur = roundIfTiny(euler[2])

            EX_inc += roundIfTiny((joystickX/100) * 2)
            EY_inc += roundIfTiny((joystickY/100) * 2)
            EZ_inc += roundIfTiny((joystickZ/100) * 2)
            EZ_Tar = roundIfTiny(EZ_Cur + EZ_inc)
            EY_Tar = roundIfTiny(EY_Cur + EY_inc)
            EX_Tar = roundIfTiny(EX_Cur + EX_inc)
//        Log.i(Debug_Tag,"EZ_Tar = $EZ_Tar, EY_Tar = $EY_Tar, EX_Tar = $EX_Tar")

            val T_inc = ik.eulerZYXToT(TCP_Cur_X_mm,TCP_Cur_Y_mm,TCP_Cur_Z_mm,EZ_Tar,EY_Tar,EX_Tar)
            val Orient_path = Orient_planner.Reorient(T, T_inc,joint_cur_angle)

            if (Orient_path.isEmpty()) {
                Log.e("JogReorient", "No IK solution for this Orient pose!")
                runOnUiThread {
                    Toast.makeText(this, "Cannot reach!", Toast.LENGTH_LONG).show()
                }
            }
            else{
                println("================================================================================")
                for ((i, joints) in Orient_path.withIndex()) {
                    Log.i("JogReorient", "Step $i : ${joints.joinToString{ "%.3f°".format(it) }}")
                }
            }
        }
        else{
            showErrorOnce("Error")
        }
//        joint_cur_angle[5] = 30.00f
//        val O = fk.forwardKinematicsDeg(joint_cur_angle)
//        val pos_O = fk.getTCPPosition(O)
//        Log.i(Debug_Tag,"TCP_Cur_X_mm = ${roundIfTiny(pos_O[0])}, " +
//                "TCP_Cur_Y_mm = ${roundIfTiny(pos_O[1])}, TCP_Cur_Z_mm = ${roundIfTiny(pos_O[2])}")

    }
    //End of Jog function

    //Begin of button function
    private fun btn_Xp(verbose: Boolean = false){
        btn_Xp.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEStopActive==false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("btn_Xp", "+100mm")
                    MoveLinearX(100.0f)
                }
            }
        }
    }
    private fun btn_Yp(verbose: Boolean = false){
        btn_Yp.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEStopActive==false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("btn_Yp", "+100mm")
                    MoveLinearY(100.0f)
                }
            }
        }
    }
    private fun btn_Zp(verbose: Boolean = false){
        btn_Zp.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEStopActive==false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("btn_Zp", "+100mm")
                    MoveLinearZ(100.0f)
                }
            }
        }
    }

    private fun btn_Xm(verbose: Boolean = false){
        btn_Xm.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEStopActive==false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("btn_Xm", "-100mm")
                    MoveLinearX(-100.0f)
                }
            }
        }
    }
    private fun btn_Ym(verbose: Boolean = false){
        btn_Ym.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEStopActive==false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("btn_Ym", "-100mm")
                    MoveLinearY(-100.0f)
                }
            }
        }
    }
    private fun btn_Zm(verbose: Boolean = false){
        btn_Zm.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEStopActive==false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("btn_Zm", "-100mm")
                    MoveLinearZ(-100.0f)
                }
            }
        }
    }

    private fun move_btn_Posi(verbose: Boolean = false){
        btnMove.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if (isTarChanged == true){
                showDialog("Press solve to calculate IK for the new point",
                    "The target has changed")
            }

            else if(isEStopActive==false && isTarChanged == false) {
                if (isEnablePressed == true){
                    if(verbose) Log.i("move_btn_Posi", "Sending new position ")
                    val debug:String = buildJogJoystickPacket(CommandFlags.Move,currentJogModeByte,
                        isEnablePressed,joint_tar_angle_ModePos,null,null,false)
                    lifecycleScope.launch(Dispatchers.IO) {
                        client.send(debug)
                        if(verbose) Log.i("Json send ", "Ena_send JogMode.Posi String: ${debug}")
                    }
                }
            }
        }
    }
    private fun solve_btn(verbose: Boolean = true){
        btnSolve.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }

            else if(isEStopActive==false) {
                if(verbose) Toast.makeText(this, "Solving IK", Toast.LENGTH_LONG).show()
                if(update_tar_coordinate()){
                    showIKsols()
                    isTarChanged = false
                }
                else{
                    Toast.makeText(this, "Invalid XYZ Coordinate", Toast.LENGTH_SHORT).show()
                }

            }
        }
    }
    private fun Estop_btn(){
        btnEstop.setOnClickListener {
            isEStopActive = !isEStopActive
            if (isEStopActive) {
                Toast.makeText(this, "E-Stop ACTIVATED! Robot stopped!", Toast.LENGTH_SHORT).show()
                showDialog("E-Stop ACTIVATED","Robot control")
                joystickX = 0.0f
                joystickY = 0.0f
                joystickZ = 0.0f
                joystick1.isEnabled = false
                joystick2.isEnabled = false
                val debug:String = buildJogJoystickPacket(CommandFlags.EmerStop,currentJogModeByte,
                    false,null,null,null,false)
                lifecycleScope.launch(Dispatchers.IO) {
                    client.send(debug)
                }

            } else {
                Toast.makeText(this, "E-Stop DEACTIVATED! Robot can move again.", Toast.LENGTH_SHORT).show()
                joystick1.isEnabled = true
                joystick2.isEnabled = true
                val debug:String = buildJogJoystickPacket(null,currentJogModeByte,
                    isEnablePressed,null,null,null,false)
                lifecycleScope.launch(Dispatchers.IO) {
                    client.send(debug)
                }
            }
        }
    }
    private fun Calib_btn(verbose: Boolean = false){
        btnCalib.setOnClickListener{
            if(verbose) Log.i("btnEstop", "Clicked")
            val debug:String = buildJogJoystickPacket(CommandFlags.Calib,currentJogModeByte,
                isEnablePressed,null,null,null,false)
            lifecycleScope.launch(Dispatchers.IO) {
                client.send(debug)
                if(verbose) Log.i("Json send ", "Calib_btn send String: ${debug}")
            }
        }
    }
    private fun Ena_btn(){
        btnEnable.setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> {
                    isEnablePressed = true
                    btnEnable.alpha = 0.6f
                    Ena_send()
                    true
                }

                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    isEnablePressed = false
                    spinnerJogMode.isEnabled = true
                    speedBar.isEnabled = true
                    btnEnable.alpha = 1.0f
                    Ena_send()
                    true
                }

                else -> true
            }
        }
    }
    private fun Home_btn(verbose: Boolean = false){
        btnHome.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to jog Home!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Return to Home position!", Toast.LENGTH_LONG).show()
                println("Jogging home")
                val debug:String = buildJogJoystickPacket(CommandFlags.MoveHome,currentJogModeByte,
                    isEnablePressed,null,null,null,false)
                lifecycleScope.launch(Dispatchers.IO) {
                    client.send(debug)
                    if(verbose) Log.i("Json send ", "Home_btn send String: ${debug}")
                }
            }
        }
    }

    private fun MovelinZ_btn(verbose: Boolean = false){
        btnMoveLinearZ.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to test move linear", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Test move linear!", Toast.LENGTH_LONG).show()
                testMoveLinearZ()
            }
        }
    }
    private fun MovelinY_btn(verbose: Boolean = false){
        btnMoveLinearY.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to test move linear", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Test move linear!", Toast.LENGTH_LONG).show()
                testMoveLinearY()
            }
        }
    }
    private fun MovelinX_btn(verbose: Boolean = false){
        btnMoveLinearX.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to test move linear", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Test move linear!", Toast.LENGTH_LONG).show()
                testMoveLinearX()
            }
        }
    }

    private fun MoveOriZ_btn(verbose: Boolean = false){
        btnMoveOrientZ.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to test move linear", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Test move linear!", Toast.LENGTH_LONG).show()
                testMoveOriZ()
            }
        }
    }
    private fun MoveOriY_btn(verbose: Boolean = false){
        btnMoveOrientY.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to test move linear", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Test move linear!", Toast.LENGTH_LONG).show()
                testMoveOriY()
            }
        }
    }
    private fun MoveOriX_btn(verbose: Boolean = false){
        btnMoveOrientX.setOnClickListener {
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to test move linear", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnClickListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                Toast.makeText(this, "Test move linear!", Toast.LENGTH_LONG).show()
//                testMoveOriX()
                testOri()
            }
        }
    }

    private fun joystickXY(verbose: Boolean = false){
        joystick1.isAutoReCenterButton = true
        joystick1.setOnMoveListener({ angle, strength ->
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnMoveListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to jog!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnMoveListener
            }
            else if((isEnablePressed) && (isEStopActive==false)) {
                var y = 0.0f
                var x = 0.0f
                if ((angle > 60)&&(angle < 120)){
                    y = strength.toFloat()
                    x = 0.0f
                    if(verbose) Log.i("joystickXY", "y = $strength, x = 0")
                }
                else if((angle > 240)&&(angle < 300)){
                    y = -strength.toFloat()
                    x = 0.0f
                    if(verbose) Log.i("joystickXY", "y = $strength, x = 0")
                }
                else if((angle > 150)&&(angle < 210)){
                    y = 0.0f
                    x = -strength.toFloat()
                    if(verbose) Log.i("joystickXY", "x = $strength, y = 0")
                }

                else if(((angle > 0)&&(angle < 30)) || ((angle > 330)&&(angle < 360))){
                    y = 0.0f
                    x = strength.toFloat()
                    if(verbose) Log.i("joystickXY", "x = $strength, y = 0")
                }
                else{
                    y = roundIfTiny((strength * cos(Math.toRadians(angle.toDouble()))).toFloat())
                    x = roundIfTiny((strength * sin(Math.toRadians(angle.toDouble()))).toFloat())
                }
                joystickX = x
                joystickY = y
                if(verbose) Log.i("joystickXY", "joystickX = $joystickX, joystickY = $joystickY")
                if (strength > 0) {
                    spinnerJogMode.isEnabled = false
                    speedBar.isEnabled = false
                    Joystick1Moving = true
                } else {
                    spinnerJogMode.isEnabled = true
                    speedBar.isEnabled = true
                    joystickX = 0.0f
                    joystickY = 0.0f
//                    println("Joystick1; X: $joystickX, Y: $joystickY")
                    Joystick1Moving = false
                    if (isRobError == true){
                        j1_total_inc = 0.0f
                        j2_total_inc = 0.0f
                        j4_total_inc = 0.0f
                        j5_total_inc = 0.0f
                    }
                }
            }
        },sampleRate)
    }
    private fun joystickZ(){
        joystick2.isAutoReCenterButton = true
        joystick2.setOnMoveListener({ angle, strength ->
            if (isEStopActive) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "E-Stop active!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnMoveListener
            }
            if (!isEnablePressed) {
                val currentTime = System.currentTimeMillis()
                if (currentTime - lastToastTime > 2000) {
                    Toast.makeText(this, "Hold ENABLE to jog!", Toast.LENGTH_SHORT).show()
                    lastToastTime = currentTime
                }
                return@setOnMoveListener
            }
            else if(isEnablePressed && (isEStopActive==false)) {
                val z = roundIfTiny((strength * Math.cos(Math.toRadians(angle.toDouble()))).toFloat())
                joystickZ = z
//                println("Joystick2; Z: $z")
                if (strength > 0) {
                    spinnerJogMode.isEnabled = false
                    speedBar.isEnabled = false
                    Joystick2Moving = true
                } else {
                    spinnerJogMode.isEnabled = true
                    speedBar.isEnabled = true
                    joystickZ = 0.0f
                    Joystick2Moving = false
                }
            }
        },sampleRate)
    }
    //End of button function

    //Begin of Test function
    private fun testInterpolate(){
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 0f, 0f)
        val thetaTest2 = floatArrayOf(0f, 10f, 20f, 30f, 50f, 80f, 90f)
        Interpolate(thetaTest,thetaTest2,true)
    }
    private fun testIK() {
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 0f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val result = ik.solveIK(T)
        val sol_Veri = ik.getVerifiedSolutions(result, T)
        for (i in sol_Veri.indices) {
            val formatted = sol_Veri[i].joinToString(", ") { String.format("%.2f°", it) }
            Log.i("IK", "SOL #$i = $formatted")
        }
    }

    private fun testFK() {
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }
    }
    private fun testOri(){
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]-0,T[1][3]-0,T[2][3],euler[0],euler[1],euler[2]-90)
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val Ori_path = L_planner.planReorientTrajectory(
            fk = fk,
            ik = ik,
            T_start = T,
            T_end   = T_p2,
            prev    = thetaTest,
            steps   = 250
        )
        InterpolateAndSend(
            jointPath = Ori_path,
            dt = 0.1f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
        println("=============================================================================")
        for ((i, joints) in Ori_path.withIndex()) {
            Log.i("Reorient_trajectory", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
    }
    private fun testPlanner() {
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]-0,T[1][3]-0,T[2][3]-300,euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 10.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
    }

    private fun testMoveLinearZ(){
//        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
//        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
//                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val Ori = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3],T[1][3],T[2][3]-300,euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 8.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory_Z", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
        InterpolateAndSend(
            jointPath = L_path,
            dt = 0.3f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
    }
    private fun testMoveLinearY(){
//        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
//        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
//                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val Ori = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3],T[1][3]-300,T[2][3],euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 8.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory_Y", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
        InterpolateAndSend(
            jointPath = L_path,
            dt = 0.3f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
    }
    private fun testMoveLinearX(){
//        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
//        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
//                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val Ori = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]-200,T[1][3],T[2][3],euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 8.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory_X", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
        InterpolateAndSend(
            jointPath = L_path,
            dt = 0.3f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
    }

    private fun MoveLinearZ(offset: Float){
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val Ori = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3],T[1][3],T[2][3]+offset,euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 10.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory_Z", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
        InterpolateAndSend(
            jointPath = L_path,
            dt = 0.3f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
    }
    private fun MoveLinearY(offset: Float){
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val Ori = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3],T[1][3]+offset,T[2][3],euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 10.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory_Y", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
        InterpolateAndSend(
            jointPath = L_path,
            dt = 0.3f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
    }
    private fun MoveLinearX(offset: Float){
        val T = fk.forwardKinematicsDeg(joint_cur_angle)
        val Ori = fk.getTCPOrientation(T)
        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]+offset,T[1][3],T[2][3],euler[0],euler[1],euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.planLinearByMM(T, T_p2, 10.0f,true,joint_cur_angle)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("L_trajectory_X", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
        InterpolateAndSend(
            jointPath = L_path,
            dt = 0.3f,
            Vstep = Vstep,
            Vezi = Vezi,
            tAccCom = tAccCom,
            mode = JogMode.Linear,
            enable = isEnablePressed
        )
    }

    private fun testMoveOriX(verbose: Boolean = true){
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]-0,T[1][3]-0,T[2][3],euler[0],euler[1],euler[2]-90)
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.Reorient(T, T_p2, thetaTest)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("Reorient_trajectory", "Step $i : " +
                    "${joints.joinToString{ "%.4f°".format(it) }}")
        }
        Interpolate(joint_cur_angle,L_path[0],verbose)
        val debug:String = buildJogJoystickPacket(CommandFlags.Move,JogMode.Linear,
            true,L_path[0],joint_tar_speed,joint_tar_accel,false)
        lifecycleScope.launch(Dispatchers.IO) {
            client.send(debug)
            if(verbose) Log.i("Json send ", "Ena_send JogMode.Angle String: ${debug}")
        }
    }
    private fun testMoveOriY(){
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]-0,T[1][3]-0,T[2][3],euler[0],euler[1]-90,euler[2])
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.Reorient(T, T_p2, thetaTest)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("Reorient_trajectory", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
    }
    private fun testMoveOriZ(){
        val thetaTest = floatArrayOf(0f, 0f, 0f, 0f, 0f, 90f, 0f)
        Log.i("theta test","from j1 to j6 = [${thetaTest[1]},${thetaTest[2]}," +
                "${thetaTest[3]},${thetaTest[4]},${thetaTest[5]},${thetaTest[6]}]" )
        val T = fk.forwardKinematicsDeg(thetaTest)
        val Ori = fk.getTCPOrientation(T)

        val euler = fk.getTCPEulerZYX(Ori)
        println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler[0], euler[1], euler[2]))

        println("=== T06 (FK at theta test) ===")
        for (i in 0..3) {
            println(T[i].joinToString("\t") { "%.3f".format(it) })
        }

        val T_p2 = ik.eulerZYXToT(T[0][3]-0,T[1][3]-0,T[2][3],euler[0]-90,euler[1],euler[2]-90)
        println("=== T06 (FK at T_p2) ===")
        for (i in 0..3) {
            println(T_p2[i].joinToString("\t") { "%.3f".format(it) })
        }

        val L_path = L_planner.Reorient(T, T_p2, thetaTest)
        println("=============================================================================")
        for ((i, joints) in L_path.withIndex()) {
            Log.i("Reorient_trajectory", "Step $i : ${joints.joinToString{ "%.4f°".format(it) }}")
        }
    }
    //End of Test function

    //----------------------------------Begin of JSON Function--------------------------------------
    private fun RequestNeededPermissions(){
        //Permission for wifi
        ActivityCompat.requestPermissions(
            this,
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
            ),
            999
        )

        //Permission for external storage
        if (hasStoragePermission()) {
            startConfigProcess()
        } else {
            requestPermissions(
                arrayOf(
                    android.Manifest.permission.WRITE_EXTERNAL_STORAGE,
                    android.Manifest.permission.READ_EXTERNAL_STORAGE
                ),
                123
            )
        }

    }
    //Check permission to access external memory
    private fun hasStoragePermission(): Boolean {
        val write = ContextCompat.checkSelfPermission(this, android.Manifest.permission.WRITE_EXTERNAL_STORAGE)
        val read = ContextCompat.checkSelfPermission(this, android.Manifest.permission.READ_EXTERNAL_STORAGE)
        return write == PackageManager.PERMISSION_GRANTED && read == PackageManager.PERMISSION_GRANTED
    }
    //Request permission to access external memory
    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == 123) {
            if (grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
                startConfigProcess()
            }
            else {
                Toast.makeText(this, "App need permission", Toast.LENGTH_SHORT).show()
                Handler(Looper.getMainLooper()).postDelayed({
                    finishAffinity()
                    exitProcess(0)
                }, 1000)
            }
        }
    }

    private fun CreateConfigFile() {
        val folder = File(
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS),
            "MyAppConfig"
        )
        if (!folder.exists()) {
            folder.mkdirs()
        }
        val file = File(folder, "config.json")
        if (!file.exists()) {
            file.writeText("{\"speed\": 100, \"mode\": \"auto\"}")
            Log.i("CONFIG", "Created new config.json")
        } else {
            Log.i("CONFIG", "Config file already exists, not overwriting")
        }
    }                     //Saving configuration file
    private fun startConfigProcess() {
        CreateConfigFile()
        ReadConfigFile()
    }                   //start Config Proces
    private fun ReadConfigFile(){
        val file = File(
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
                .toString() + "/MyAppConfig/config.json"
        )

        if (file.exists()) {
            val text = file.readText()
            Log.i("CONFIG", text)
        }
    }                        //Read Config file
    private fun writeConfigFile(content: String) {
        val folder = File(
            Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS),
            "MyAppConfig"
        )
        if (!folder.exists()) {
            folder.mkdirs()
            Log.i("CONFIG", "Created folder MyAppConfig")
        }

        val file = File(folder, "config.json")
        file.writeText(content)
        Log.i("CONFIG", "Config content saved: $content")
    }       //Write Config file

    //Nếu app này được cài đặt lên máy mới thì file json mẫu sẽ được copy vô bộ nhớ trong của máy
    private fun copyJsonToInternalStorage(context: Context, fileName: String) {
        val file = File(context.filesDir, fileName)
        if (!file.exists()) {
            try {
                context.assets.open(fileName).use { inputStream ->
                    file.outputStream().use { outputStream ->
                        inputStream.copyTo(outputStream)
                    }
                }
                Log.i(Debug_Tag, "Copied file $fileName to internal storage")
            }
            catch (ex: IOException) {
                Log.e(Debug_Tag, "Error copying file $fileName", ex)
            }
        }
        else {
            Log.i(Debug_Tag, " File $fileName existed")
        }
    }
    private fun readJson(context: Context, fileName: String): String? {
        return try {
            val file = File(context.filesDir, fileName)
            file.readText()
        } catch (e: Exception) {
            e.printStackTrace()
            Log.e("JSON_READ", "Lỗi đọc file $fileName: ${e.message}")
            null
        }
    }
    private fun loadJoints_Json(context: Context): JointsData? {
        return try {
            val file = File(context.filesDir, "Joints_send.json")
            if (!file.exists()) {
                copyJsonToInternalStorage(context,"Joints_send.json")
            }
            val jsonString = file.readText()
            gson.fromJson(jsonString, JointsData::class.java)
        } catch (e: Exception) {
            e.printStackTrace()
            Log.e(Debug_Tag, "Error reading Joints_send.json from internal storage")
            null
        }
    }
    //------------------------------------End of JSON Function--------------------------------------

    //UPDATE UI
    private fun updateUIAngles(angleArr: FloatArray, verbose: Boolean = true) {
        if (verbose) Log.i(Debug_Tag,"Begin updateUIAngles")
        if (isRobError == false){
            if (angleArr.size < 7) return

            else {
                txtView_J1_cur_angle.apply {
                    text = String.format("%.2f", angleArr[1])
                    setTextColor(Color.BLACK)
                }

                txtView_J2_cur_angle.apply {
                    text = String.format("%.2f", angleArr[2])
                    setTextColor(Color.BLACK)
                }

                txtView_J3_cur_angle.apply {
                    text = String.format("%.2f", angleArr[3])
                    setTextColor(Color.BLACK)
                }

                txtView_J4_cur_angle.apply {
                    text = String.format("%.2f", angleArr[4])
                    setTextColor(Color.BLACK)
                }

                txtView_J5_cur_angle.apply {
                    text = String.format("%.2f", angleArr[5])
                    setTextColor(Color.BLACK)
                }

                txtView_J6_cur_angle.apply {
                    text = String.format("%.2f", angleArr[6])
                    setTextColor(Color.BLACK)
                }
            }
        }
        else {
            if (angleArr.size < 7) return
            else {
                txtView_J1_cur_angle.apply {
                    text = String.format("%.2f", angleArr[1])
                    setTextColor(Color.RED)
                }

                txtView_J2_cur_angle.apply {
                    text = String.format("%.2f", angleArr[2])
                    setTextColor(Color.RED)
                }

                txtView_J3_cur_angle.apply {
                    text = String.format("%.2f", angleArr[3])
                    setTextColor(Color.RED)
                }

                txtView_J4_cur_angle.apply {
                    text = String.format("%.2f", angleArr[4])
                    setTextColor(Color.RED)
                }

                txtView_J5_cur_angle.apply {
                    text = String.format("%.2f", angleArr[5])
                    setTextColor(Color.RED)
                }

                txtView_J6_cur_angle.apply {
                    text = String.format("%.2f", angleArr[6])
                    setTextColor(Color.RED)
                }
            }
        }
        if (verbose) Log.i(Debug_Tag,"End updateUIAngles")
    }
    private fun updateJointHomeIcon(imageView: ImageView, jointStatus: Int, isHome: Int, verbose: Boolean = true) {
        if (verbose) Log.i(Debug_Tag,"jointStatus = ${jointStatus}")

        when (jointStatus) {
            StatusFlags.Error.toInt(),
            StatusFlags.LIM.toInt(),
            StatusFlags.LIE.toInt(),
            StatusFlags.OOR.toInt(),
            StatusFlags.OOS.toInt() -> {
                imageView.setImageResource(R.drawable.svg_error_flag)
                if (verbose) Log.i(Debug_Tag,"jointStatus = svg_error_flag")
                if (isHome == 1){
                    imageView.setImageResource(R.drawable.svg_home_flag)
                    if (verbose) Log.i(Debug_Tag,"jointStatus = svg_home_flag")
                }
            }

            StatusFlags.Busy.toInt() -> {
                imageView.setImageResource(R.drawable.svg_busy_flag)
                if (verbose) Log.i(Debug_Tag,"jointStatus = svg_busy_flag")
            }
            StatusFlags.Complete.toInt() -> {
                imageView.setImageResource(R.drawable.svg_complete_flag)
                if (isHome == 1){
                    imageView.setImageResource(R.drawable.svg_home_flag)
                    if (verbose) Log.i(Debug_Tag,"jointStatus = svg_home_flag")
                }
            }
            StatusFlags.Calibrated.toInt() -> {
                imageView.setImageResource(R.drawable.svg_complete_flag)
                if (isHome == 1){
                    imageView.setImageResource(R.drawable.svg_home_flag)
                    if (verbose) Log.i(Debug_Tag,"jointStatus = svg_home_flag")
                }
            }

            else -> imageView.setImageDrawable(null)
        }
    }
    private fun showDialog(Error: String?,ClasName: String?){
        val dialog= Dialog(this)
        dialog.requestWindowFeature(Window.FEATURE_NO_TITLE)
        dialog.setCancelable(false)
        dialog.setContentView(R.layout.layout_custom_dialog)
        dialog.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        val tvMess: TextView = dialog.findViewById(R.id.tv_message)
        val btnAck: Button = dialog.findViewById(R.id.btnAck)
        val tvNameFunc: TextView = dialog.findViewById(R.id.tv_nameFunc)
        tvNameFunc.text = ClasName
        tvMess.text = Error
        btnAck.setOnClickListener{
            dialog.dismiss()
        }
        dialog.show()
    }
    private fun showIKsols(verbose: Boolean = false){
        ikDialog?.dismiss()
        ikDialog = null
        ikDialog = Dialog(this)
        val dialog = ikDialog!!

        val T_test = ik.eulerZYXToT(Euler_tar_XYZ_ModePos[0],Euler_tar_XYZ_ModePos[1],Euler_tar_XYZ_ModePos[2],
            Euler_tar_angle_ModePos[0],Euler_tar_angle_ModePos[1],Euler_tar_angle_ModePos[2])
        if(verbose){
            println("=== T06 (FK at T_test) ===")
            Log.i("","Ez = ${Euler_tar_angle_ModePos[0]}, Ey = ${Euler_tar_angle_ModePos[1]}, " +
                    "Ex = ${Euler_tar_angle_ModePos[2]}, ")
            for (i in 0..3) {
                println(T_test[i].joinToString("\t") { "%.3f".format(it) })
            }
        }
        val result = ik.solveIK(T_test)
        if(verbose){
            for (i in result.indices) {
                val formatted = result[i].joinToString(", ") { String.format("%.2f°", it) }
                Log.i("IK", "result #$i = $formatted")
            }
        }
        val sol_Veri = ik.getVerifiedSolutions(result, T_test)
        if (verbose){
            for (i in sol_Veri.indices) {
                val formatted = sol_Veri[i].joinToString(", ") { String.format("%.2f°", it) }
                Log.i("IK", "sol_Veri #$i = $formatted")
            }
        }

        dialog.requestWindowFeature(Window.FEATURE_NO_TITLE)
        dialog.setCancelable(false)
        dialog.setContentView(R.layout.solution_layout)
        dialog.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog.window?.apply {
            setLayout(
                WindowManager.LayoutParams.WRAP_CONTENT,
                WindowManager.LayoutParams.WRAP_CONTENT
            )
            addFlags(WindowManager.LayoutParams.FLAG_NOT_TOUCH_MODAL)
            addFlags(WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE)
            clearFlags(WindowManager.LayoutParams.FLAG_DIM_BEHIND)
        }
        val spinnerSols: Spinner  = dialog.findViewById(R.id.spinnerSols)
        val tvNameFunc: TextView = dialog.findViewById(R.id.tv_nameFunc)
        val btnAck: Button = dialog.findViewById(R.id.btnAck)
        if (sol_Veri.size > 0){
            tvNameFunc.text = "Total solutions: ${sol_Veri.size}"
            btnAck.setOnClickListener{
                dialog.dismiss()
                ikDialog = null
            }
            dialog.show()
            /*val spinnerItems: List<String> = sol_Veri.mapIndexed { solIndex, solution ->
                val angles = (1..6).joinToString(", ") { jointIndex ->
                    String.format("%.2f", solution[jointIndex])
                }

                "Solution ${solIndex + 1}: $angles"
            }*/
            val spinnerItems: List<String> = sol_Veri.mapIndexed { solIndex, solution ->
                val firstLine = (1..3).joinToString(", ") {
                    String.format("%.2f°", solution[it])
                }
                val secondLine = (4..6).joinToString(", ") {
                    String.format("%.2f°", solution[it])
                }
                "Solution ${solIndex + 1}:\n$firstLine,\n$secondLine"
            }

            val adapter = ArrayAdapter(
                this,
                R.layout.spinner_layout,
                spinnerItems
            )
            adapter.setDropDownViewResource(R.layout.spinner_layout)
            spinnerSols.adapter = adapter

            spinnerSols.onItemSelectedListener =
            object : AdapterView.OnItemSelectedListener {

                override fun onItemSelected(
                    parent: AdapterView<*>,
                    view: View?,
                    position: Int,
                    id: Long
                ) {
                    val selectedSolution = sol_Veri[position]

                    // vì solution[0] bỏ, dùng 1..6
                    val j1 = roundIfTiny(selectedSolution[1])
                    val j2 = roundIfTiny(selectedSolution[2])
                    val j3 = roundIfTiny(selectedSolution[3])
                    val j4 = roundIfTiny(selectedSolution[4])
                    val j5 = roundIfTiny(selectedSolution[5])
                    val j6 = roundIfTiny(selectedSolution[6])
                    joint_tar_angle_ModePos [1] = j1
                    joint_tar_angle_ModePos [2] = j2
                    joint_tar_angle_ModePos [3] = j3
                    joint_tar_angle_ModePos [4] = j4
                    joint_tar_angle_ModePos [5] = j5
                    joint_tar_angle_ModePos [6] = j6
                    if(verbose) Log.i("IK", "Selected solution #${position + 1}: " +
                            "$j1, $j2, $j3, $j4, $j5, $j6")
                }

                override fun onNothingSelected(parent: AdapterView<*>) {}
            }

            val dragView: View = dialog.findViewById(R.id.tv_nameFunc)
            var lastX = 0
            var lastY = 0
            dragView.setOnTouchListener { _, event ->
                val params = dialog.window?.attributes ?: return@setOnTouchListener false

                when (event.action) {
                    MotionEvent.ACTION_DOWN -> {
                        lastX = event.rawX.toInt()
                        lastY = event.rawY.toInt()
                    }

                    MotionEvent.ACTION_MOVE -> {
                        val dx = event.rawX.toInt() - lastX
                        val dy = event.rawY.toInt() - lastY

                        params.x += dx
                        params.y += dy

                        dialog.window?.attributes = params

                        lastX = event.rawX.toInt()
                        lastY = event.rawY.toInt()
                    }
                }
                true
            }

        }
        else {
            showDialog("No solutions","Out of reach")
        }

    }
    private fun showErrorOnce(msg: String) {
        if (isErrorDialogShowing) return
        isErrorDialogShowing = true
        runOnUiThread {
            showDialog(msg,"Robot hardware")
        }
    }
    //----------------------------------Begin of Command func--------------------------------------
    private fun buildJointsSendPacket(
        command: String,
        mode: String,
        enable: Int,
        chunkId: Int,
        totalChunks: Int,
        speed: Float,
        acceleration: Float,
        trajectoryList: List<FloatArray>     // mỗi phần tử là float[7]
    ): String {

        val root = JSONObject()

        root.put("command", command)
        root.put("mode", mode)
        root.put("enable", enable)
        root.put("chunk_id", chunkId)
        root.put("total_chunks", totalChunks)

        val control = JSONObject()
        control.put("speed", speed)
        control.put("acceleration", acceleration)
        root.put("control", control)

        val trajArray = JSONArray()

        for (j in trajectoryList) {

            // bảo vệ: đảm bảo array có đúng 7 phần tử
            if (j.size != 7) {
                throw IllegalArgumentException("Trajectory pose must have 7 elements (0..6)")
            }

            val poseObj = JSONObject()

            val jointsArray = JSONArray()

            // ---- LẤY TỪ INDEX 1..6 ----
            for (i in 1..6) {
                jointsArray.put(j[i])
            }

            poseObj.put("joints", jointsArray)
            trajArray.put(poseObj)
        }

        root.put("trajectory", trajArray)

        return root.toString()
    }

    private fun buildJogJoystickPacket(
        FlagCommand:Byte?,
        mode: Byte,
        enable: Boolean,
        jointsAngle: FloatArray?,
        jointsSpeed: FloatArray?,
        jointsAccel: FloatArray?,
        wrapWithMarkers: Boolean = true
    ): String {

        var jsonString: String = " "
        if (FlagCommand!= null){
            if (jointsAngle != null) {
                val root = JSONObject().apply {
                    put("command", FlagCommand)
                    put("mode", mode)
                    put("enable", if (enable) 1 else 0) // keep as int for compatibility; could be put("enable", enable) for boolean
                }
                val formattedAngles = Array(7) { i -> "%.3f".format(jointsAngle[i]) }
                val jArray = JSONArray()
                for (i in 1..6) {
                    jArray.put(formattedAngles[i].toFloat())
                }
                root.put("angle", jArray)
                if (jointsSpeed != null){
                    val formattedSpeed = Array(7) { i -> "%.2f".format(jointsSpeed[i]) }
                    val jArray2 = JSONArray()
                    for (i in 1..6) {
                        jArray2.put(formattedSpeed[i].toFloat())
                    }
                    root.put("speed", jArray2)
                }
                if (jointsAccel!= null){
                    val formattedAccel = Array(7) { i -> "%.2f".format(jointsAccel[i]) }
                    val jArray3 = JSONArray()
                    for (i in 1..6) {
                        jArray3.put(formattedAccel[i].toFloat())
                    }
                    root.put("accel", jArray3)
                }
                jsonString = root.toString()
            }

            else {
                val root = JSONObject().apply {
                    put("command", FlagCommand)
                    put("mode", mode)
                    put("enable", if (enable) 1 else 0) // keep as int for compatibility; could be put("enable", enable) for boolean
                }
                jsonString = root.toString()
            }
        }
        else{
            if (jointsAngle != null) {
                val root = JSONObject().apply {
                    if (isRobError == true){
                        put("command", CommandFlags.Move_Offset)
                    }
                    else{
                        put("command", CommandFlags.MoveJoystick)
                    }
                    put("mode", mode)
                    put("enable", if (enable) 1 else 0) // keep as int for compatibility; could be put("enable", enable) for boolean
                }
                val formattedAngles = Array(7) { i -> "%.3f".format(jointsAngle[i]) }
                val jArray = JSONArray()
                for (i in 1..6) {
                    jArray.put(formattedAngles[i].toFloat())
                }
                root.put("angle", jArray)
                if (jointsSpeed != null){
                    val formattedSpeed = Array(7) { i -> "%.2f".format(jointsSpeed[i]) }
                    val jArray2 = JSONArray()
                    for (i in 1..6) {
                        jArray2.put(formattedSpeed[i].toFloat())
                    }
                    root.put("speed", jArray2)
                }

                if (jointsAccel!= null){
                    val formattedAccel = Array(7) { i -> "%.2f".format(jointsAccel[i]) }
                    val jArray3 = JSONArray()
                    for (i in 1..6) {
                        jArray3.put(formattedAccel[i].toFloat())
                    }
                    root.put("accel", jArray3)
                }

                jsonString = root.toString()
            }

            else if(jointsAngle == null ){
                val root = JSONObject().apply {
//                    if (isRobError == true){
//                        put("command", CommandFlags.Standby)
//                    }
//                    else{
//                        put("command", CommandFlags.MoveJoystick)
//                    }
                    put("command", CommandFlags.Standby)
                    put("mode", mode)
                    put("enable", if (enable) 1 else 0) // keep as int for compatibility; could be put("enable", enable) for boolean
                }
                jsonString = root.toString()
            }
        }

        return if (wrapWithMarkers) "<START>$jsonString<STOP>"
                else jsonString
    }

    private fun Ena_send(verbose:Boolean = true){
        when (currentJogModeByte) {
            JogMode.Angle -> {
                if (isEnablePressed == true){
                    Interpolate(joint_cur_angle,joint_tar_angle_ModeAngle,true)
                    val debug:String = buildJogJoystickPacket(CommandFlags.Move,JogMode.Angle,
                        true,joint_tar_angle_ModeAngle,joint_tar_speed,joint_tar_accel,false)
                    lifecycleScope.launch(Dispatchers.IO) {
                        client.send(debug)
                        if(verbose) Log.i("Json send ", "Ena_send JogMode.Angle String: ${debug}")
                    }
                }
                else{
                    val debug:String = buildJogJoystickPacket(CommandFlags.Standby,currentJogModeByte,
                        false,null,null,null,false)
                    lifecycleScope.launch(Dispatchers.IO) {
                        client.send(debug)
                        if(verbose) Log.i("Json send ", "Ena_send standby String: ${debug}")
                    }
                }

            }
/*
            JogMode.Posi -> {
                if (isEnablePressed == true){
                    val debug:String = buildJogJoystickPacket(CommandFlags.Move,currentJogModeByte,
                        isEnablePressed,joint_tar_angle_ModeAn, nullgle,null,false)
                    lifecycleScope.launch(Dispatchers.IO) {
                        client.send(debug)
                        if(verbose) Log.i("Json send ", "Ena_send JogMode.Posi String: ${debug}")
                    }
                }
                else{
                    val debug:String = buildJogJoystickPacket(CommandFlags.Standby,currentJogModeByte,
                        isEnablePressed,null,null,false)
, null                    lifecycleScope.launch(Dispatchers.IO) {
                        client.send(debug)
                        if(verbose) Log.i("Json send ", "Ena_send standby String: ${debug}")
                    }
                }
            }*/

            else -> {
                val debug:String = buildJogJoystickPacket(CommandFlags.Standby,currentJogModeByte,
                    isEnablePressed,null, null,null,false)
                lifecycleScope.launch(Dispatchers.IO) {
                    client.send(debug)
                    if(verbose) Log.i("Json send ", "Ena_send String: ${debug}")
                }
            }
        }

        /*if (currentJogModeByte != JogMode.Angle){
            val debug:String = buildJogJoystickPacket(CommandFlags.Standby,currentJogModeByte,
                isEnablePressed,null,null,false)
, null            lifecycleScope.launch(Dispatchers.IO) {
                client.send(debug)
                if(verbose) Log.i("Json send ", "Ena_send String: ${debug}")
            }
        }
        else {
            val debug:String = buildJogJoystickPacket(CommandFlags.Move,currentJogModeByte,
                isEnablePressed,joint_tar_angle_ModeAn, nullgle,null,false)
            lifecycleScope.launch(Dispatchers.IO) {
                client.send(debug)
                if(verbose) Log.i("Json send ", "Ena_send JogMode.Angle String: ${debug}")
            }
        }*/
    }
}

