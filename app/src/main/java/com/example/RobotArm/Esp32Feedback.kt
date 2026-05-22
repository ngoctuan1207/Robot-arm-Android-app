package com.example.RobotArm

data class Esp32Feedback(
    val command: Int,
    val mode: Int,
    val enable: Int,
    val rob_status: Int,
    val current: CurrentData
)

data class CurrentData(
    val angle: FloatArray,
    val velo: FloatArray,
    val stat: IntArray,
    val home: IntArray
)