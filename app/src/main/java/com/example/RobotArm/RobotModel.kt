package com.example.RobotArm

data class RobotModel(
    val dhParams: List<DHParam>,
    val flange: DHParam,
    val baseXOffset: Float,
    val jointDirection: FloatArray
)

data class DHParam(
    val a: Float,
    val d: Float,
    val alpha: Float,
    val thetaOffset: Float
)