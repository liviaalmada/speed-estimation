package graphhopper;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.SQLException;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.Month;
import java.time.ZoneId;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
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
import com.graphhopper.util.shapes.GHPoint;
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

	public static class SpeedMatch {
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

	public static Map<Integer, SpeedMatch> estimateSpeed(List<EdgeMatch> matches) {
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

	public static double msTokmh(double speed) {
		return speed * 3.6;
	}

	public static Coordinate[] getCoordinates(PointList g) {
		Coordinate[] coords = new Coordinate[g.size()];

		for (int i = 0; i < coords.length; i++) {
			coords[i] = new Coordinate(g.getLon(i), g.getLat(i));
		}

		return coords;
	}

	public static void doSaveMapMatching(FileWriter writer, Map<Integer, SpeedMatch> mapLinkToSpeed, Integer trajId) {
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

	public static Map<Integer, List<GPXEntry>> matchingTDriveFile(String path) {
		Map<Integer, List<GPXEntry>> trajectories = new HashMap<>();
		try {
			FileReader reader = new FileReader(new File(path));
			// 1;2008-02-02 15:36:08;116.51172;39.92123
			BufferedReader buffer = new BufferedReader(reader);
			String line;

			DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss", Locale.ENGLISH);
			ArrayList<GPXEntry> list = null;
			int idTraj = -1;
			long previousMills = 0;
			long millis = 0;
			while ((line = buffer.readLine()) != null) {
				String[] values = line.split(";");
				int idTrajNext = Integer.parseInt(values[0]);
				LocalDateTime dateTime = LocalDateTime.parse(values[1], formatter);
				double lon = Double.parseDouble(values[2]);
				double lat = Double.parseDouble(values[3]);
				previousMills = millis;
				millis = dateTime.toInstant(ZoneOffset.ofTotalSeconds(0)).toEpochMilli();
				GPXEntry gpxEntry = new GPXEntry(new GHPoint(lat, lon), millis);

				if (idTraj == -1) {
					idTraj = idTrajNext;
					list = new ArrayList<>();
					previousMills = millis;
				} else {
					if (idTraj != idTrajNext || (millis - previousMills) > 3600000 ) {
						
						RunGraphHopperMapMatchingSpeed.doMatching(list);

						// RunGraphHopperMapMatchingSpeed.doSaveMapMatching(writer,
						// estimatedSpeed, entry.getKey());

						idTraj = idTrajNext;
						// trajectories.put(idTraj, list);
						list = new ArrayList<>();
						idTraj = -1;
					}
				}

				list.add(gpxEntry);

			}
			buffer.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return trajectories;
	}

	public static Map<Integer, MatchResult> doMatching(Timestamp startDatetime, Timestamp endDatetime,
			Map<Integer, List<GPXEntry>> trajectories) {
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
			}

		}

		return results;

	}

	public static void doMatching(List<GPXEntry> trajectory) {
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

		Map<Integer, MatchResult> results = new HashMap<>();

		log.info("started!");
		try {
			MatchResult mr = mapMatching.doWork(trajectory);
			RunGraphHopperMapMatchingSpeed.estimateSpeed(mr.getEdgeMatches());
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
