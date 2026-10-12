package com.example.pointcloudmapper.spatial

import com.example.pointcloudmapper.model.ColoredPoint
import java.util.concurrent.ConcurrentHashMap
import kotlin.math.floor

/**
 * Key representing a 3D grid cell for spatial deduplication.
 */
data class VoxelKey(val x: Int, val y: Int, val z: Int)

/**
 * Manages a persistent 3D point cloud tracking points by ARCore feature Point ID and voxel grid filtering.
 *
 * @param voxelSize The size of each 3D voxel cubic cell in meters (e.g. 0.02f = 2cm).
 */
class VoxelGridPointCloudManager(val voxelSize: Float = 0.02f) {

    // Map feature point ID to current ColoredPoint
    private val pointsById = ConcurrentHashMap<Int, ColoredPoint>()

    /**
     * Converts world 3D coordinates into voxel key indices.
     */
    fun getVoxelKey(x: Float, y: Float, z: Float): VoxelKey {
        return VoxelKey(
            floor(x / voxelSize).toInt(),
            floor(y / voxelSize).toInt(),
            floor(z / voxelSize).toInt()
        )
    }

    /**
     * Adds or updates a point by its unique ARCore feature ID.
     */
    fun addPoint(point: ColoredPoint): Boolean {
        val existing = pointsById[point.id]
        if (existing == null || point.confidence >= existing.confidence) {
            pointsById[point.id] = point
            return true
        }
        return false
    }

    /**
     * Adds a list of points.
     */
    fun addPoints(points: List<ColoredPoint>): Int {
        var addedCount = 0
        for (p in points) {
            if (addPoint(p)) {
                addedCount++
            }
        }
        return addedCount
    }

    /**
     * Returns all accumulated unique feature points.
     */
    fun getPoints(): List<ColoredPoint> {
        return pointsById.values.toList()
    }

    /**
     * Returns the total number of points in the point cloud.
     */
    fun pointCount(): Int {
        return pointsById.size
    }

    /**
     * Clears all stored points.
     */
    fun clear() {
        pointsById.clear()
    }
}
