package com.example.pointcloudmapper.model

import java.io.Serializable

/**
 * Represents a 3D point with spatial coordinates (x, y, z), RGB colors (0..255),
 * ARCore feature point ID, and confidence score.
 */
data class ColoredPoint(
    val id: Int,
    val x: Float,
    val y: Float,
    val z: Float,
    val r: Int,
    val g: Int,
    val b: Int,
    val confidence: Float = 1.0f
) : Serializable
