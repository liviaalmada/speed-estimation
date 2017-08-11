package graphhopper;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.SQLException;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.Month;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.logging.Logger;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.postgresql.jdbc.TimestampUtils;
import org.slf4j.LoggerFactory;

import com.graphhopper.GraphHopper;
import com.graphhopper.matching.EdgeMatch;
import com.graphhopper.matching.GPXExtension;
import com.graphhopper.matching.LocationIndexMatch;
import com.graphhopper.matching.MapMatching;
import com.graphhopper.matching.MatchResult;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.GPXEntry;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint3D;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;

import br.ufc.arida.dao.TrajectoryPointDAO;
import br.ufc.arida.mapmatching.Edge;

public class RunGraphHopperMapMatchingSpeed {

	private static final CarFlagEncoder encoder = new CarFlagEncoder();
	private static GraphHopper hopper = new GraphHopper();
	protected static final Logger LOGGER = Logger.getGlobal();
	private static org.slf4j.Logger log = LoggerFactory.getLogger(RunGraphHopperMapMatchingSpeed.class);
	private static GeometryFactory geoFactory = JTSFactoryFinder.getGeometryFactory();

	private static class SpeedMatch {
		EdgeMatch edgeMacth;
		double speed;
		double timestamp;
		double travelTime;
	}

	public static void doImport(String osmFilePath, String graphHopperLocation) {
		hopper.setOSMFile(osmFilePath);
		hopper.setGraphHopperLocation(graphHopperLocation);
		hopper.setEncodingManager(new EncodingManager(encoder));
		hopper.setCHEnable(false);
		hopper.importOrLoad();
	}

	public static boolean isPointInLine(Point point, LineString line) {
		if (getDistanceInMeters(line.distance(point)) < 0.1)
			return true;
		return false;
	}

	public static double getDistanceInMeters(double angularDistance) {
		return angularDistance * (Math.PI / 180) * 6378137;
	}

	private static Map<Integer, SpeedMatch> estimateSpeed(List<EdgeMatch> matches) {
		List<EdgeMatch> linksToDistributeSpeed = new ArrayList<>();
		Map<Integer, SpeedMatch> mapLinkToSpeed = new HashMap<>();
		double totalLenght = 0, endDelta = 0;
		boolean firstFound = false, lastFound = false;
		GHPoint3D gpsLast = null, gpsFirst = null;
		Point last = null, first = null;
		long timeFirst = 0, timeLast = 0;
		for (int i = 0; i < matches.size(); i++) {
			EdgeMatch edgeMatch = matches.get(i);

			// Get gps points in the actual edge
			List<GPXExtension> gpsCorrected = edgeMatch.getGpxExtensions();
			// Get edge geometry
			PointList geometry = edgeMatch.getEdgeState().fetchWayGeometry(3);
			Coordinate[] coords = getCoordinates(geometry);
			linksToDistributeSpeed.add(edgeMatch);

			if (gpsCorrected.size() <= 1) {
				LineString lineString = geoFactory.createLineString(coords);
				totalLenght += getDistanceInMeters(lineString.getLength());
				continue;
			}

			if (!firstFound) {
				gpsFirst = gpsCorrected.get(0).getQueryResult().getSnappedPoint();
				timeFirst = gpsCorrected.get(0).getEntry().getTime();
				first = geoFactory.createPoint(new Coordinate(gpsFirst.lon, gpsFirst.lat));
			}

			if (!lastFound) {
				gpsLast = gpsCorrected.get(gpsCorrected.size() - 1).getQueryResult().getSnappedPoint();
				last = geoFactory.createPoint(new Coordinate(gpsLast.lon, gpsLast.lat));
				timeLast = gpsCorrected.get(gpsCorrected.size() - 1).getEntry().getTime();
			}

			for (int j = 0; j < coords.length - 1; j++) {
				LineString lineString = geoFactory.createLineString(Arrays.copyOfRange(coords, j, j + 2));
				if (!firstFound) {
					if (isPointInLine(first, lineString)) {
						firstFound = true;
						totalLenght += getDistanceInMeters(first.distance(lineString.getEndPoint()));
					}
				}

				if (firstFound && !lastFound && gpsLast != null) {
					if (isPointInLine(last, lineString)) {
						lastFound = true;
						// end delta do ponto de gps até o final da linha
						endDelta += getDistanceInMeters(last.distance(lineString.getEndPoint()));

						// total do início da linha até o ponto de gps
						totalLenght += getDistanceInMeters(lineString.getStartPoint().distance(last));

					} else {
						totalLenght += getDistanceInMeters(lineString.getLength());
					}
				} else {
					endDelta += getDistanceInMeters(lineString.getLength());
				}
			}

			if (firstFound && lastFound) {
				double speed = msTokmh(totalLenght / (timeLast - timeFirst) * 1000);
				linksToDistributeSpeed.add(edgeMatch);
				double nextTimestamp = -1;

				for (EdgeMatch edge : linksToDistributeSpeed) {
					SpeedMatch speedMatch = new SpeedMatch();
					speedMatch.edgeMacth = edgeMatch;
					speedMatch.speed = speed;

					if (nextTimestamp == -1) {
						speedMatch.timestamp = edgeMatch.getGpxExtensions().get(0).getEntry().getTime();
						GHPoint3D point = edgeMatch.getGpxExtensions().get(0).getQueryResult().getSnappedPoint();
						Point p = geoFactory.createPoint(new Coordinate(point.lat, point.lon));
						PointList pointList = edgeMatch.getEdgeState().fetchWayGeometry(3);
						double distanceInMeters = getDistanceInMeters(p.distance(
								geoFactory.createPoint(new Coordinate(pointList.getLongitude(pointList.getSize() - 1),
										pointList.getLatitude(pointList.getSize() - 1)))));
						speedMatch.travelTime = distanceInMeters / speed;
						nextTimestamp = speedMatch.timestamp + speedMatch.travelTime;
					} else {
						speedMatch.timestamp = nextTimestamp;
						PointList pointList = edgeMatch.getEdgeState().fetchWayGeometry(3);
						Point p1 = geoFactory
								.createPoint(new Coordinate(pointList.getLongitude(0), pointList.getLatitude(0)));
						Point p2 = geoFactory
								.createPoint(new Coordinate(pointList.getLongitude(pointList.getSize() - 1),
										pointList.getLatitude(pointList.getSize() - 1)));
						double distanceInMeters = getDistanceInMeters(p1.distance(p2));

						speedMatch.travelTime = distanceInMeters / speed;
						nextTimestamp = speedMatch.timestamp + speedMatch.travelTime;
					}

					mapLinkToSpeed.put(edge.getEdgeState().getEdge(), speedMatch);
				}
				linksToDistributeSpeed.clear();
				totalLenght = endDelta;
				endDelta = 0;
				firstFound = true;
				lastFound = false;
				timeFirst = timeLast;
				gpsFirst = gpsLast;
			}

		}

		return mapLinkToSpeed;
	}

	private static double msTokmh(double speed) {
		return speed * 3.6;
	}

	private static Coordinate[] getCoordinates(PointList g) {
		Coordinate[] coords = new Coordinate[g.size()];

		for (int i = 0; i < coords.length; i++) {
			coords[i] = new Coordinate(g.getLon(i), g.getLat(i));
		}

		return coords;
	}

	private static void doSaveMapMatching(FileWriter writer, Map<Integer, SpeedMatch> mapLinkToSpeed, Integer trajId) {
		try {

			for (Entry<Integer, SpeedMatch> e : mapLinkToSpeed.entrySet()) {
				SpeedMatch speedMath = e.getValue();
				writer.write(trajId + "," + speedMath.edgeMacth.getEdgeState().getEdge() + ","
						+ (int) (speedMath.timestamp / 1000) + "," + speedMath.speed + "\n");
			}

		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	private static Map<Integer, MatchResult> doMatching(Timestamp startDatetime, Timestamp endDatetime) {
		// create MapMatching object, can and should be shared across
		// threads
		GraphHopperStorage graph = hopper.getGraphHopperStorage();
		FlagEncoder firstEncoder = hopper.getEncodingManager().fetchEdgeEncoders().get(0);
		int gpxAccuracy = 110;
		LocationIndexTree paramIndex = (LocationIndexTree) hopper.getLocationIndex();
		LocationIndexMatch locationIndex = new LocationIndexMatch(graph, paramIndex, gpxAccuracy);
		MapMatching mapMatching = new MapMatching(graph, locationIndex, firstEncoder);
		mapMatching.setMaxVisitedNodes(1000);
		mapMatching.setForceRepair(true);
		TrajectoryPointDAO dao = new TrajectoryPointDAO();
		

		try {
			Map<Integer, List<GPXEntry>> trajectories = dao.readTrajectoriesAsGPXEntries("taxi_junho", startDatetime,
					endDatetime);
			Map<Integer, MatchResult> results = new HashMap<>();

			for (Entry<Integer, List<GPXEntry>> entry : trajectories.entrySet()) {
				List<GPXEntry> trajs = entry.getValue();
				// do the actual matching, get the GPX entries
				// from a file or via stream
				try {
					log.info("started!");
					MatchResult mr = mapMatching.doWork(trajs);
					results.put(entry.getKey(), mr);
					// return GraphHopper edges with all
					// associated GPX entries

				} catch (Exception e) {
					log.info("Invalid!");
					//e.printStackTrace();
				}

			}

			return results;

		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		// }

		// }

		return null;

	}

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
			
			doImport(osm, graphHopper);
			ZoneId zoneId = ZoneId.of("GMT-3");
			int year = 2016;
			for (int month = 6; month < 7; month++) {
				FileWriter writer = new FileWriter(new File("map-matching-output-teste"+month+"-"+year));
				for (int day = 1; day < 2; day++) {
					try {
						LocalDate d = LocalDate.of(year, month, day);
						ZonedDateTime zdt = ZonedDateTime.of(d, LocalTime.of(startHour, 00, 00), zoneId);
						ZonedDateTime zdt2 = ZonedDateTime.of(d, LocalTime.of(endHour, 59, 59), zoneId);
						java.sql.Timestamp startDatetime = java.sql.Timestamp.from(zdt.toInstant());
						java.sql.Timestamp endDatetime = java.sql.Timestamp.from(zdt2.toInstant());
						
						LOGGER.info("Getting trajectories from " + startDatetime + " to " + endDatetime);
						
						Map<Integer, MatchResult> doMatching = doMatching(startDatetime, endDatetime);
						for (Entry<Integer, MatchResult> entry : doMatching.entrySet()) {
							MatchResult matchResult = entry.getValue();
							Map<Integer, SpeedMatch> estimatedSpeed = estimateSpeed(matchResult.getEdgeMatches());
							doSaveMapMatching(writer, estimatedSpeed, entry.getKey());
						}
					}catch (java.time.DateTimeException e) {
						LOGGER.info("Dia inválido");
					}
					
					
				}
				writer.close();
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
