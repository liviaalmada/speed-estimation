package graphhopper;

import java.util.List;
import java.util.Map;

import com.graphhopper.util.GPXEntry;

public class RunMapMatchingBeijing {
	public static void main(String[] args) {
	

			int startDay = 1;
			int endDay = 2; // including start day
							// and
							// excluding
			// end day
			int startHour = 0; // Integer.parseInt(args[2]),
			int endHour = 23; // Integer.parseInt(args[3]); // including
								// startHour
								// and
			// Read osm file as a Graph
			String osm = "/Users/liviaalmada/Documents/osm-fortaleza.osm";
			String graphHopper = "ghnetwork";
			// try {

			// RunGraphHopperMapMatchingSpeed.doImport(osm, graphHopper);
			// ZoneId zoneId = ZoneId.of("GMT-3");
			// int year = 2016;
			// TrajectoryPointDAO dao = new TrajectoryPointDAO();

			Map<Integer, List<GPXEntry>> trajectories = RunGraphHopperMapMatchingSpeed
					.matchingTDriveFile("/media/livia/DATA/DADOS/Beijing/tdrive.csv");
	}
}
