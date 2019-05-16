/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.List;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.INodeCollisionChecker;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

public class MockCollisionChecker implements INodeCollisionChecker {

    Node ncvStartNode, ncvEndNode;
    
    @Override
    public boolean hasCollision(List<Node> trajectory, double timeOffset, double distanceOffset) {
        Node p1 = trajectory.get(0);
        Node q1 = trajectory.get(1);
        Node p2 = ncvStartNode;
        Node q2 = ncvEndNode;
        if(p1 == null || q1 == null || p2 == null || q2 == null) return false;
        int o1 = orientation(p1, q1, p2); 
        int o2 = orientation(p1, q1, q2); 
        int o3 = orientation(p2, q2, p1); 
        int o4 = orientation(p2, q2, q1);
        if (o1 != o2 && o3 != o4) return true;  
        if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
        if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
        if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        return false;
    }
    
    public void setPredictedTrajectory(Node startNode, Node endNode) {
        ncvStartNode = startNode;
        ncvEndNode = endNode;
    }
    
    
    private boolean onSegment(Node p, Node q, Node r) {
        return q.getTime()     <= Math.max(p.getTime(), r.getTime())     &&
               q.getTime()     >= Math.min(p.getTime(), r.getTime())     &&
               q.getDistance() <= Math.max(p.getDistance(), r.getDistance()) &&
               q.getDistance() >= Math.min(p.getDistance(), r.getDistance());
    }
    
    private int orientation(Node p, Node q, Node r) {
        int val = (int) ((q.getDistance() - p.getDistance()) * (r.getTime()     - q.getTime()) -
                  (q.getTime()     - p.getTime())     * (r.getDistance() - q.getDistance()));
        if(val == 0) return val;
        return val > 0 ? 1 : 2;
    }
    
}
