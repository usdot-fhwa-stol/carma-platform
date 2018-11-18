package gov.dot.fhwa.saxton.carma.signal_plugin;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.util.Arrays;
import java.util.List;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.SimpleNCVMotionPredictor;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.AStarSolver;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.NeighborBase;

public class AStarSolverTest {
	
	// Treat a Node as a 2-D grid map for testing a star solver
	private class SimpleGridCost implements ICostModel {
		Node goal_ = null;
		@Override
		public double cost(Node n1, Node n2) {
			return Math.abs(n1.getDistance() - n2.getDistance()) + Math.abs(n1.getTime() - n2.getTime());
		}
		@Override
		public double heuristic(Node n1) {
			//(goal_.getDistance() - n1.getDistance()) + (goal_.getTime() - n1.getTime())
			return 0;
		}
		@Override
		public void setTolerances(Node tolerances) { }
		@Override
		public void setGoal(Node goal) { goal_ = goal; }
		@Override
		public boolean isGoal(Node n) {
			return n.getDistance() == goal_.getDistance() && n.getTime() == goal_.getTime();
		}
		@Override
		public boolean isUnusable(Node n) {
			return n.getDistance() > 0 && n.getTime() == 2;
		}
	}
	
	private class SimpleGridNeighborGenerator extends NeighborBase {
		@Override
		public List<Node> neighbors(Node node) {
			long x = node.getDistance(), y = node.getTime();
			Node up = new Node(x, y + 1, 0);
			Node down = new Node(x, y - 1, 0);
			Node left = new Node(x - 1, y, 0);
			Node right = new Node(x + 1, y, 0);
			return Arrays.asList(up, down, left, right);
		}
	}
	
	@Before
	public void setUp() throws Exception {
		ILoggerFactory mockFact = mock(ILoggerFactory.class, Mockito.withSettings().stubOnly());
        ILogger mockLogger = mock(ILogger.class, Mockito.withSettings().stubOnly());
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
	}
	
	@Test
	public void testSimpleAStarPlan() {
		ICostModel costModel = new SimpleGridCost();
		costModel.setGoal(new Node(4, 4, 0));
		AStarSolver solver = new AStarSolver();
		List<Node> path = solver.solve(new Node(0, 0, 0), costModel, new SimpleGridNeighborGenerator());
		for(Node n : path) System.out.println(n);
	}
	
}
