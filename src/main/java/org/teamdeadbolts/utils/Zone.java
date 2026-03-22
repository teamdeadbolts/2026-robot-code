/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a polygonal zone on the field. Used to check if the robot is within
 * specific spatial boundaries (e.g., scoring zones, hazard areas).
 */
public class Zone {
    private final ArrayList<Translation2d> vertices;
    private double minX = Double.MAX_VALUE;
    private double maxX = Double.MIN_VALUE;
    private double minY = Double.MAX_VALUE;
    private double maxY = Double.MIN_VALUE;

    private int t = 0;

    /**
     * @param vertices The vertices defining the polygon of the zone.
     */
    public Zone(Translation2d... vertices) {
        this.vertices = new ArrayList<>();
        for (Translation2d vertex : vertices) {
            this.vertices.add(vertex);
        }
        calculateMinMax();
    }

    /**
     * Updates the zone's vertices and recalculates its bounding box.
     * @param vertices The new vertices for the zone.
     */
    public void setVertices(Translation2d... vertices) {
        this.vertices.clear();
        for (Translation2d vertex : vertices) {
            this.vertices.add(vertex);
        }
        calculateMinMax();
    }

    /**
     * Determines if a point is within the zone using the Ray Casting algorithm.
     * Includes a bounding box pre-check for performance.
     * * @param point The point to check.
     * @return True if the point is inside the polygon.
     */
    public boolean contains(Translation2d point) {
        // https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
        // Bounding box check for early exit
        if (point.getX() < minX || point.getX() > maxX || point.getY() < minY || point.getY() > maxY) {
            return false;
        }

        boolean inside = false;
        for (int i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) {
            if ((vertices.get(i).getY() > point.getY()) != (vertices.get(j).getY() > point.getY())
                    && point.getX()
                            < (vertices.get(j).getX() - vertices.get(i).getX())
                                            * (point.getY() - vertices.get(i).getY())
                                            / (vertices.get(j).getY()
                                                    - vertices.get(i).getY())
                                    + vertices.get(i).getX()) {
                inside = !inside;
            }
        }

        return inside;
    }

    /** @return The list of vertices defining this zone. */
    public ArrayList<Translation2d> getVertices() {
        return vertices;
    }

    /**
     * Visualizes the zone's vertices in AdvantageScope by cycling through them.
     */
    public void visulize() {
        int i = t++ % vertices.size();
        Pose2d pose = new Pose2d(vertices.get(i), new Rotation2d());
        synchronized (Logger.class) {
            Logger.recordOutput("Debug/ZonePose", pose);
        }
    }

    /**
     * Calculates the axis-aligned bounding box for the polygon.
     */
    private void calculateMinMax() {
        minX = Double.MAX_VALUE;
        maxX = Double.MIN_VALUE;
        minY = Double.MAX_VALUE;
        maxY = Double.MIN_VALUE;

        for (Translation2d vertex : vertices) {
            if (vertex.getX() < minX) minX = vertex.getX();
            if (vertex.getX() > maxX) maxX = vertex.getX();
            if (vertex.getY() < minY) minY = vertex.getY();
            if (vertex.getY() > maxY) maxY = vertex.getY();
        }
    }
}
