/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class Zone {
    private ArrayList<Translation2d> vertices;
    private double minX = Double.MAX_VALUE;
    private double maxX = Double.MIN_VALUE;
    private double minY = Double.MAX_VALUE;
    private double maxY = Double.MIN_VALUE;

    /**
     * Create a new zone
     *
     * @param vertices The vertices of the zone
     */
    public Zone(Translation2d... vertices) {
        this.vertices = new ArrayList<>();
        for (Translation2d vertex : vertices) {
            if (vertex.getX() < minX) {
                minX = vertex.getX();
            }
            if (vertex.getX() > maxX) {
                maxX = vertex.getX();
            }
            if (vertex.getY() < minY) {
                minY = vertex.getY();
            }
            if (vertex.getY() > maxY) {
                maxY = vertex.getY();
            }
            this.vertices.add(vertex);
        }
    }

    // https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    public boolean contains(Translation2d point) {
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

    public ArrayList<Translation2d> getVertices() {
        return vertices;
    }
}
