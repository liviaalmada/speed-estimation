package graphhopper;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.SQLException;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.graphhopper.matching.MatchResult;
import com.graphhopper.util.GPXEntry;

import br.ufc.arida.dao.TrajectoryPointDAO;
import graphhopper.RunGraphHopperMapMatchingSpeed.*;

public class MapmatchingTaxisimplesMain {
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
		try {

			RunGraphHopperMapMatchingSpeed.doImport(osm, graphHopper);
			ZoneId zoneId = ZoneId.of("GMT-3");
			int year = 2016;
			for (int month = 6; month < 7; month++) {
				FileWriter writer = new FileWriter(new File("map-matching-output-teste" + month + "-" + year));
				for (int day = 1; day < 2; day++) {
					try {
						LocalDate d = LocalDate.of(year, month, day);
						ZonedDateTime zdt = ZonedDateTime.of(d, LocalTime.of(startHour, 00, 00), zoneId);
						ZonedDateTime zdt2 = ZonedDateTime.of(d, LocalTime.of(endHour, 59, 59), zoneId);
						java.sql.Timestamp startDatetime = java.sql.Timestamp.from(zdt.toInstant());
						java.sql.Timestamp endDatetime = java.sql.Timestamp.from(zdt2.toInstant());
						TrajectoryPointDAO dao = new TrajectoryPointDAO();
						Map<Integer, List<GPXEntry>> trajectories = dao .readTrajectoriesAsGPXEntriesTSDatabase("taxi_junho", startDatetime,
								endDatetime);
						RunGraphHopperMapMatchingSpeed.LOGGER.info("Getting trajectories from " + startDatetime + " to " + endDatetime);

						Map<Integer, MatchResult> doMatching = RunGraphHopperMapMatchingSpeed.doMatching(startDatetime, endDatetime,trajectories);
						for (Entry<Integer, MatchResult> entry : doMatching.entrySet()) {
							MatchResult matchResult = entry.getValue();
							Map<Integer, SpeedMatch> estimatedSpeed = RunGraphHopperMapMatchingSpeed.estimateSpeed(matchResult.getEdgeMatches());
							RunGraphHopperMapMatchingSpeed.doSaveMapMatching(writer, estimatedSpeed, entry.getKey());
						}
					} catch (java.time.DateTimeException e) {
						RunGraphHopperMapMatchingSpeed.LOGGER.info("Dia inválido");
					} catch (ClassNotFoundException e) {
						e.printStackTrace();
					} catch (SQLException e) {
						e.printStackTrace();
					}

				}
				writer.close();
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
