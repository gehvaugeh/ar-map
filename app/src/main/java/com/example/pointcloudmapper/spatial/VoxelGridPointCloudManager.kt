package com.example.pointcloudmapper.spatial

import com.example.pointcloudmapper.model.ColoredPoint
import java.util.concurrent.ConcurrentHashMap
import kotlin.math.floor

/**
 * Key representing a 3D grid cell for spatial deduplication.
 */
data class VoxelKey(val x: Int, val y: Int, val z: Int)

/**
 * Manages a persistent 3D point cloud with voxel grid spatial deduplication.
 *
 * @param voxelSize The size of each 3D voxel cubic cell in meters (e.g. 0.02f = 2cm).
 */
class VoxelGridPointCloudManager(val voxelSize: Float = 0.02f) {

    private val grid = ConcurrentHashMap<VoxelKey, ColoredPoint>()

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
     * Adds a point if the voxel cell is empty or if the new point has higher confidence.
     * Returns true if a new point was added or updated.
     */
    fun addPoint(point: ColoredPoint): Boolean {
        val key = getVoxelKey(point.x, point.y, point.z)
        val existing = grid[key]
        if (existing == null || point.confidence > existing.confidence) {
            grid[key] = point
            return true
        }
        return false
    }

    /**
     * Adds a list of points to the grid.
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
     * Returns all accumulated points.
     */
    fun getPoints(): List<ColoredPoint> {
        return grid.values.toList()
    }

    /**
     * Returns the total number of points in the point cloud.
     */
    fun pointCount(): Int {
        return grid.size
    }

    /**
     * Clears all stored points.
     */
    fun clear() {
        grid.clear()
    }
}
