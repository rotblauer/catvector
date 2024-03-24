/*
Package main is a program which takes a stream of geojson point features
and outputs a single geojson feature, a linestring.
The important logic is in its aggregations of point->linestring properties.

Use:

  zcat ~/tdata/edge.json.gz | tail -1000 | catvector

Experimental logic is implemented for Kalman filtering and smoothing,
trying out different libraries and parameters.
*/

package main

import (
	"bufio"
	"bytes"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"sync"
	"time"

	"github.com/davecgh/go-spew/spew"
	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/planar"
	"github.com/paulmach/orb/simplify"
	rkalman "github.com/regnull/kalman"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

func init() {
	log.SetFlags(log.Lshortfile | log.LstdFlags)
}

const (
	earthRadius                       = 6378137.0 // meters
	earthCircumference                = math.Pi * earthRadius * 2
	earthCircumferenceMetersPerDegree = earthCircumference / 360
	earthCircumferenceDegreesPerMeter = 360 / earthCircumference
)

var flagDwellInterval = flag.Duration("dwell-interval", 10*time.Second, "stop detection interval")
var flagTripStartInterval = flag.Duration("trip-start-interval", 30*time.Second, "start detection interval")
var flagTrackerSpeedThreshold = flag.Float64("speed-threshold", 0.5, "speed threshold for trip detection")
var flagDwellDistanceThresholdDefault = 10.0
var flagDwellDistanceThreshold = flag.Float64("cluster-distance", flagDwellDistanceThresholdDefault, "cluster distance threshold for trip stops")

func cmdPointsToLineStrings() {
	trackerWaiter := sync.WaitGroup{}
	uuidTrackers := map[string]*PointTracker{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, nil)
loop:
	for {
		select {
		case f := <-featureCh:
			tracker, ok := uuidTrackers[f.Properties["UUID"].(string)]
			if !ok {
				tracker = NewPointTracker(*flagDwellInterval)
				uuidTrackers[f.Properties["UUID"].(string)] = tracker
				go func() {
					trackerWaiter.Add(1)
					defer trackerWaiter.Done()
					for ls := range tracker.linestringsCh {
						j, err := ls.MarshalJSON()
						if err != nil {
							log.Fatalln(err)
						}
						j = append(j, []byte("\n")...)
						if _, err := os.Stdout.Write(j); err != nil {
							log.Fatalln(err)
						}
					}
				}()
			}
			tracker.AddPointFeature(f)
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			// Iterate all trackers and flush them.
			for _, tracker := range uuidTrackers {
				tracker.Flush()
			}
			break loop
		}
	}
	trackerWaiter.Wait()
}

func cmdLinestringsToPoints() {
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, nil)
loop:
	for {
		select {
		case f := <-featureCh:
			center := f.Geometry.(orb.LineString).Bound().Center()
			point := geojson.NewFeature(orb.Point(center))
			point.Properties = f.Properties
			j, err := point.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := os.Stdout.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
}

var flagDouglasPeuckerThreshold = flag.Float64("threshold", 0.0001, "Douglas-Peucker epsilon threshold")

func cmdDouglasPeucker() {
	log.Println("Douglas-Peucker simplification with threshold", *flagDouglasPeuckerThreshold)
	simplifier := simplify.DouglasPeucker(*flagDouglasPeuckerThreshold)
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, func(f *geojson.Feature) (*geojson.Feature, error) {
		// The feature must be a linestring feature.
		if _, ok := f.Geometry.(orb.LineString); !ok {
			return nil, errors.New("not a linestring")
		}
		ls := geojson.LineString(f.Geometry.(orb.LineString))
		simplerGeometry := simplifier.Simplify(ls.Geometry())
		f.Geometry = simplerGeometry
		return f, nil
	})
loop:
	for {
		select {
		case f := <-featureCh:
			if f == nil {
				continue
			}
			j, err := f.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := os.Stdout.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
}

func cmdValidate() {
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, func(f *geojson.Feature) (*geojson.Feature, error) {
		if err := validatePointFeature(f); err != nil {
			return nil, err
		}
		return f, nil
	})
loop:
	for {
		select {
		case f := <-featureCh:
			if f == nil {
				continue
			}
			j, err := f.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := os.Stdout.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
}

func featureMustBeLinestring(f *geojson.Feature) (*geojson.Feature, error) {
	if _, ok := f.Geometry.(orb.LineString); !ok {
		return nil, errors.New("not a linestring")
	}
	return f, nil
}

type LapperNapperTracker struct {
	CurrentSpeed    float64
	CurrentActivity Activity
	CurrentLocation *geojson.Feature

	CurrentFeature *geojson.Feature
	FeatureCh      chan *geojson.Feature
}

func (t *LapperNapperTracker) AddLinestring(f *geojson.Feature) {
	if t.CurrentFeature == nil {
		t.CurrentSpeed = f.Properties["AverageReportedSpeed"].(float64)
		t.CurrentActivity = ActivityFromReport(f.Properties["Activity"])
		if f.Properties["Duration"].(float64) > 60 {
			// Current location is last point of linestring
			t.CurrentLocation = geojson.NewFeature(f.Geometry.(orb.LineString)[len(f.Geometry.(orb.LineString))-1])
		} else {
			// Current location is centroid of linestring
			t.CurrentLocation = geojson.NewFeature(f.Geometry.(orb.LineString).Bound().Center())
		}
		t.CurrentFeature = f
		return
	}

}

func NewLapperNapperTracker() *LapperNapperTracker {
	return &LapperNapperTracker{
		FeatureCh: make(chan *geojson.Feature),
	}
}

func (t *LapperNapperTracker) Flush() {
	if t.CurrentFeature != nil {
		t.FeatureCh <- t.CurrentFeature
	}
	close(t.FeatureCh)
}

// cmdLapsOrNaps is a program which takes a stream of geojson linestring features
// and decides to either possibly join them (if moving, aka lapping), or to
// convert them to points and possibly join them if napping.
func cmdLapsOrNaps() {
	waiter := sync.WaitGroup{}
	uuidTrackers := map[string]*LapperNapperTracker{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, featureMustBeLinestring)
loop:
	for {
		select {
		case f := <-featureCh:
			tracker, ok := uuidTrackers[f.Properties["UUID"].(string)]
			if !ok {
				tracker = NewLapperNapperTracker()
				uuidTrackers[f.Properties["UUID"].(string)] = tracker
				go func() {
					waiter.Add(1)
					defer waiter.Done()
					for outFeature := range tracker.FeatureCh {
						j, err := outFeature.MarshalJSON()
						if err != nil {
							log.Fatalln(err)
						}
						j = append(j, []byte("\n")...)
						if _, err := os.Stdout.Write(j); err != nil {
							log.Fatalln(err)
						}
					}
				}()
			}
			tracker.AddLinestring(f)
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			// Iterate all trackers and flush them.
			for _, tracker := range uuidTrackers {
				tracker.Flush()
			}
			break loop
		}
	}
	waiter.Wait()
}

func cmdTripDetector() {
	uuidTripDetectors := map[string]*TripDetector{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, nil)
loop:
	for {
		select {
		case f := <-featureCh:
			if f == nil {
				panic("nil feature")
			}
			td, ok := uuidTripDetectors[f.Properties.MustString("UUID")]
			if !ok {
				td = NewTripDetector(*flagDwellInterval, *flagTripStartInterval, *flagTrackerSpeedThreshold, *flagDwellDistanceThreshold)
				uuidTripDetectors[f.Properties.MustString("UUID")] = td
			}
			if err := td.AddFeature(f); err != nil {
				log.Fatalln(err)
			}
			f.Properties["IsTrip"] = td.Tripping

			j, err := json.Marshal(f)
			if err != nil {
				log.Fatalln(err)
			}
			j = bytes.ReplaceAll(j, []byte("\\n"), []byte(""))
			j = bytes.ReplaceAll(j, []byte("\n"), []byte(""))
			// j = append(j, []byte("\n")...)
			// if _, err := os.Stdout.Write(j); err != nil {
			// 	log.Fatalln(err)
			// }
			if _, err := fmt.Fprintln(os.Stdout, string(j)); err != nil {
				log.Fatalln(err)
			}
			// if err := os.Stdout.Sync(); err != nil {
			// 	log.Fatalln(err)
			// }
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
}

func preProcessFilters(f *geojson.Feature) (*geojson.Feature, error) {
	// 1. NSAT: number of satellites > 4
	//    Don't have this info :(

	// 2. Altitude filter: -10 < altitude < 15000. (Commercial flight cruising altitude commonly 33000..(42000 ft == 12800 m))
	// Sorry to all you high-flying high-flyers out there.
	// > The Earth's lowest land elevation point is at the Dead Sea, located at the border of Israel and Jordan. Its shores have an elevation of 420 meters (1,385 feet) below sea level.
	// > https://www.nationalgeographic.org/encyclopedia/elevation/
	// Don't dive in the Red Sea or your cattrack won't work. Not worth it.
	if f.Properties.MustFloat64("Elevation") < -450 || f.Properties.MustFloat64("Elevation") > 13000 {
		return nil, fmt.Errorf("altitude out of range: %f", f.Properties.MustFloat64("Elevation"))
	}

	// 3. Urban canyon filter.

	// 4. Speed filter.
	// Assume less than the speed of sound. Sorry to all you speed demons out there.
	if f.Properties.MustFloat64("Speed") > 343 {
		return nil, fmt.Errorf("speed out of range: %f", f.Properties.MustFloat64("Speed"))
	}
	return f, nil
}

// urbanCanyonFilterStream removes spurious GPS readings caused by the urban canyon effect.
// > Wang: Third, GPS points away from
// the adjacent points due to the signal shift caused by
// blocking or ‘‘urban canyon’’ effect are also deleted. As
// is shown in Figure 2, GPS points away from both the
// before and after 5 points center for more than 200 m
// should be considered as shift points.
func urbanCanyonFilterStream(featuresCh chan *geojson.Feature, closingCh chan struct{}) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	// ! Wang says 200, but I got no hits with that on Rye in batch 42.
	spuriousDistanceMeters := *flagUrbanCanyonDistance
	buffer, bufferSize := []*geojson.Feature{}, 11 // 11 = 5 + 1 + 5

	go func() {
		for {
			select {
			case f := <-featuresCh:
				buffer = append(buffer, f)
				if len(buffer) < bufferSize {
					// The FIRST 5 points get automatically flushed without filtering
					// because there are no head points to compare them against.
					if len(buffer) <= 5 {
						featureChan <- f
					}
					continue
				}
				// If we've reached the buffer size, we can start processing.
				// Truncate the buffer to the last 11 elements.
				if len(buffer) > bufferSize {
					buffer = buffer[len(buffer)-bufferSize:]
				}
				// [inclusive:exclusive)
				tail := buffer[0:5]
				target := buffer[5]
				head := buffer[6:]

				// Find the centroid of the tail.
				tailCenter, _ := planar.CentroidArea(orb.MultiPoint{tail[0].Point(), tail[1].Point(), tail[2].Point(), tail[3].Point(), tail[4].Point()})
				// Find the centroid of the head.
				headCenter, _ := planar.CentroidArea(orb.MultiPoint{head[0].Point(), head[1].Point(), head[2].Point(), head[3].Point(), head[4].Point()})
				// If the distances from the target to the tail and head centroids are more than 200m, it's a shift point.
				if planar.Distance(tailCenter, target.Point()) > spuriousDistanceMeters && planar.Distance(headCenter, target.Point()) > spuriousDistanceMeters {
					j, _ := json.Marshal(target)
					errChan <- fmt.Errorf("urban canyon spurious point: %s", string(j))
					continue
				} else {
					featureChan <- target
				}

			case x := <-closingCh:
				// We never filled the buffer, flush all.
				// This indicates a noop filter because of an insufficient number of points.
				if len(buffer) > 5 && len(buffer) < bufferSize {
					for _, f := range buffer[5:] {
						featureChan <- f
					}
				} else {
					// Else we met the buffer size, but (always) the tailing 5 points get flushed
					// without filtering because there are no tail points to compare them against.
					for _, f := range buffer[6:] {
						featureChan <- f
					}
				}
				closeCh <- x
				return
			}
		}
	}()
	return featureChan, errChan, closeCh
}

var flagUrbanCanyonDistance = flag.Float64("urban-canyon-distance", 200.0, "urban canyon distance threshold")

func cmdPreprocess() {
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, preProcessFilters)
	processedCh, procErrCh, doneCh := urbanCanyonFilterStream(featureCh, closeCh)

loop:
	for {
		select {
		case f := <-processedCh:
			if f == nil {
				continue
			}
			j, err := f.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := os.Stdout.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case err := <-procErrCh:
			log.Println(err)
		case <-doneCh:
			break loop
		}
	}
}

type StopConsolidator struct {
	StopDistanceThreshold float64
	LastFeature           *geojson.Feature
	Features              TracksGeoJSON
	StopPoint             TrackGeoJSON
	stopPointN            int
}

func (sc *StopConsolidator) NewStopPoint() TrackGeoJSON {
	stopPoint := geojson.NewFeature(orb.Point{})
	sc.stopPointN++
	stopPoint.Properties = map[string]interface{}{
		"ID":       sc.stopPointN,
		"Time":     time.Time{},
		"Duration": 0.0,
		"Count":    0,
		"MaxDist":  0.0,
		"P50Dist":  0.0,
		"P99Dist":  0.0,
	}
	return TrackGeoJSON{stopPoint}
}

func NewStopConsolidator(stopDistanceThreshold float64) *StopConsolidator {
	sc := &StopConsolidator{
		StopDistanceThreshold: stopDistanceThreshold,
		stopPointN:            0,
	}
	sc.Features = TracksGeoJSON{}
	sc.StopPoint = sc.NewStopPoint()
	return sc
}

func (sc *StopConsolidator) Reset() {
	sc.Features = TracksGeoJSON{}
	sc.StopPoint = sc.NewStopPoint()
}

func (sc *StopConsolidator) MergeStopPoint(f *geojson.Feature) {
	sc.Features = append(sc.Features, &TrackGeoJSON{f})
	defer func() { sc.LastFeature = f }()

	sc.StopPoint.Properties["Name"] = f.Properties.MustString("Name")
	sc.StopPoint.Properties["UUID"] = f.Properties.MustString("UUID")
	sc.StopPoint.Properties["Version"] = f.Properties.MustString("Version")

	// TODO: Synthesize?
	sc.StopPoint.Properties["Activity"] = f.Properties.MustString("Activity")
	sc.StopPoint.Properties["Speed"] = f.Properties.MustFloat64("Speed")
	sc.StopPoint.Properties["Elevation"] = f.Properties.MustFloat64("Elevation")
	sc.StopPoint.Properties["Heading"] = f.Properties.MustFloat64("Heading")
	sc.StopPoint.Properties["Accuracy"] = f.Properties.MustFloat64("Accuracy")

	// Duration is the time between the first and last point in the stop.
	// The first feature time difference will == 0, subsequent features will have a duration.
	if sc.LastFeature != nil {
		sc.StopPoint.Feature.Properties["Duration"] =
			sc.StopPoint.Feature.Properties.MustFloat64("Duration") +
				mustGetTime(f, "Time").Sub(mustGetTime(sc.LastFeature, "Time")).
					Round(time.Second).Seconds()
	} else {
		sc.StopPoint.Feature.Properties["Duration"] = 0.0
	}

	// Merge the stop point.
	// Time is the last time.
	sc.StopPoint.Feature.Properties["Time"] = f.Properties.MustString("Time")

	// Count is the number of points in the stop.
	sc.StopPoint.Feature.Properties["Count"] = sc.StopPoint.Feature.Properties.MustInt("Count") + 1

	// MaxDist is the distance of the furthest point from the center.
	// P50 and P99 are the p50 and p99 values of the points' distances, indicating the point density.
	if len(sc.Features) < 2 {
		sc.StopPoint.Geometry = f.Point()
		sc.StopPoint.Feature.Properties["P50Dist"] = 0.0
		sc.StopPoint.Feature.Properties["P99Dist"] = 0.0
		sc.StopPoint.Feature.Properties["Area"] = 0.0
		return
	}
	lats, lngs := []float64{}, []float64{}
	points := []orb.Point{}
	for _, f := range sc.Features {
		points = append(points, f.Point())
		lats = append(lats, f.Point().Lat())
		lngs = append(lngs, f.Point().Lon())
	}
	// Get the average lat/lng.
	meanLat, _ := stats.Mean(lats)
	meanLng, _ := stats.Mean(lngs)
	center := orb.Point{meanLng, meanLat}
	sc.StopPoint.Geometry = center

	distances := []float64{}
	for _, f := range sc.Features {
		distances = append(distances, geo.Distance(center, f.Point()))
	}
	distP50, _ := stats.Percentile(distances, 50)
	distP99, _ := stats.Percentile(distances, 99)
	sc.StopPoint.Feature.Properties["P50Dist"] = distP50
	sc.StopPoint.Feature.Properties["P99Dist"] = distP99

	mp := orb.MultiPoint(points)
	sc.StopPoint.Feature.Properties["Area"] = geo.Area(mp.Bound())
}

// AddFeature adds a point feature to the StopConsolidator, a state machine.
// If the SC is empty, it will be initialized with the feature, and
// we are assumed
func (sc *StopConsolidator) AddFeature(f *geojson.Feature) TrackGeoJSON {
	if len(sc.Features) == 0 {
		sc.MergeStopPoint(f)
		return sc.StopPoint
	}
	// If the feature is not chronological, reset the state.
	if !mustGetTime(sc.LastFeature, "Time").Before(mustGetTime(f, "Time")) {
		sc.Reset()
		return sc.AddFeature(f)
	}

	// If the feature is not within the stop distance threshold, reset the state.
	if geo.Distance(sc.StopPoint.Point(), f.Point()) > sc.StopDistanceThreshold {
		sc.Reset()
		return sc.AddFeature(f)
	}

	sc.MergeStopPoint(f)
	return sc.StopPoint
}

// cmdConsolidateStops is a program which takes a stream of geojson point features
// and outputs a stream of geojson point features with stops consolidated.
// It clusters points into a single stop if they are within a certain time and space threshold.
// The Time threshold parameter is reused from --dwell-time.
// Any two points separated by this time are considered potential stops.
// Any two consecutive points separated by 100m are likewise considered potential stops.
// Successive points not meeting these conditions are grouped into a synthesized
// STOP POINT. The synthesized Stop Point's coordinates are the centroid of the points.
// The Stop Point will list Time and Duration as properties, where Time is the time of the last point,
// and Duration is the time between the first and last point in the stop.
// The Stop Point will also list the number of points in the stop as a property,
// and the distance of the furthest point from the centroid, as well
// as the p50 and p99 values of the points' distances, indicating the point density.
// Point density might be interesting to explore later, potentially for use with other stuff,
// like stop detection.
func cmdConsolidateStops() {
	uuidTrackers := map[string]*StopConsolidator{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, nil)
	lastStop := TrackGeoJSON{}
	flushPoint := func(feature *geojson.Feature) {
		j, err := feature.MarshalJSON()
		if err != nil {
			log.Fatalln(err, spew.Sdump(feature))
		}
		j = append(j, []byte("\n")...)
		if _, err := os.Stdout.Write(j); err != nil {
			log.Fatalln(err)
		}
	}
loop:
	for {
		select {
		case f := <-featureCh:
			tracker, ok := uuidTrackers[f.Properties["UUID"].(string)]
			if !ok {
				tracker = NewStopConsolidator(*flagClusterDistanceThreshold)
				uuidTrackers[f.Properties["UUID"].(string)] = tracker
			}
			stopPoint := tracker.AddFeature(f)
			if lastStop.Feature == nil {
				lastStop = stopPoint
			}

			// If the last stop is not the same as the current stop, flush the last stop.
			if lastStop.Feature.Properties.MustInt("ID") != stopPoint.Feature.Properties.MustInt("ID") {
				flushPoint(lastStop.Feature)
				lastStop = stopPoint
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			for _, tracker := range uuidTrackers {
				flushPoint(tracker.StopPoint.Feature)
			}
			break loop
		}
	}
}

func main() {
	flag.Parse()
	command := flag.Arg(0)
	switch command {
	case "validate":
		cmdValidate()
		return
	case "rkalman":
		cmdRKalmanFilter()
		return
	case "points-to-linestrings":
		cmdPointsToLineStrings()
		return
	case "trip-detector":
		cmdTripDetector()
		return
	case "laps-or-naps":
		cmdLapsOrNaps()
		return
	case "linestrings-to-points":
		cmdLinestringsToPoints()
		return
	case "douglas-peucker":
		cmdDouglasPeucker()
		return
	case "preprocess":
		cmdPreprocess()
		return
	case "consolidate-stops":
		cmdConsolidateStops()
		return
	default:
		log.Fatalf("unknown command: %s", command)
	}
}

func cmdRKalmanFilter() {
	bwriter := bufio.NewWriter(os.Stdout)
	featureCh, errCh, closeCh := readStreamRKalmanFilter(os.Stdin)

loop:
	for {
		select {
		case feature := <-featureCh:
			j, err := feature.MarshalJSON()
			if err != nil {
				dump := spew.Sdump(feature)
				log.Println(err, dump)
				continue
			}
			j = append(j, []byte("\n")...)
			if _, err := bwriter.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}

	if err := bwriter.Flush(); err != nil {
		log.Fatalln(err)
	}
}

func readStreamRKalmanFilter(reader io.Reader) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	uuidFilters := map[string]*RKalmanFilterT{}
	initFilterFor := func(pointFeature *geojson.Feature) {
		filter := &RKalmanFilterT{}
		if err := filter.InitFromPoint(pointFeature); err != nil {
			errChan <- err
			return
		}
		// Otherwise, we've just inited the Kalman filter for this UUID.
		// Since its the filter's initial point, there's no sense in estimating from one point.
		uuidFilters[pointFeature.Properties["UUID"].(string)] = filter
	}
	breader := bufio.NewReader(reader)

	go func() {
		var lastFeature *geojson.Feature
		for {
			read, err := breader.ReadBytes('\n')
			if err != nil {
				if errors.Is(err, os.ErrClosed) || errors.Is(err, io.EOF) {
					closeCh <- struct{}{}
					return
				}
				errChan <- err
			}
			pointFeature, err := geojson.UnmarshalFeature(read)
			if err != nil {
				errChan <- err
			}

			featureChronological := true
			if lastFeature != nil {
				featureChronological = mustGetTime(lastFeature, "Time").Before(mustGetTime(pointFeature, "Time"))
			}
			lastFeature = pointFeature

			filter, ok := uuidFilters[pointFeature.Properties["UUID"].(string)]
			if !ok || !featureChronological {
				initFilterFor(pointFeature)
				featureChan <- pointFeature
				continue
			}

			estimate, err := filter.EstimateFromObservation(pointFeature)
			if err != nil {
				initFilterFor(pointFeature)
				errChan <- err
				continue
			}
			featureChan <- estimate
		}
	}()

	return featureChan, errChan, closeCh
}

type RKalmanFilterT struct {
	ProcessNoise       *rkalman.GeoProcessNoise
	Filter             *rkalman.GeoFilter
	LastFeature        *geojson.Feature
	DebugLastFeatures  []*geojson.Feature
	DebugLastEstimates []*geojson.Feature
}

func (f *RKalmanFilterT) InitFromPoint(obs *geojson.Feature) error {
	f.DebugLastFeatures = []*geojson.Feature{obs}
	f.DebugLastEstimates = []*geojson.Feature{}
	f.LastFeature = obs
	speed := math.Max(1, obs.Properties["Speed"].(float64))

	// Estimate process noise.
	f.ProcessNoise = &rkalman.GeoProcessNoise{
		// We assume the measurements will take place at the approximately the
		// same location, so that we can disregard the earth's curvature.
		BaseLat: obs.Point().Lat(),
		// How much do we expect the user to move, meters per second.
		DistancePerSecond: speed, //  ls.Properties["Speeds"].([]float64)[0],
		// How much do we expect the user's speed to change, meters per second squared.
		SpeedPerSecond: math.Sqrt(speed),
	}

	// Initialize Kalman filter.
	var err error
	f.Filter, err = rkalman.NewGeoFilter(f.ProcessNoise)
	if err != nil {
		return err
	}
	return nil
}

func (f *RKalmanFilterT) EstimateFromObservation(obs *geojson.Feature) (estimatedFeature *geojson.Feature, err error) {
	f.DebugLastFeatures = append(f.DebugLastFeatures, obs)
	if len(f.DebugLastFeatures) > 60 {
		f.DebugLastFeatures = f.DebugLastFeatures[1:]
	}
	defer func() {
		if r := recover(); r != nil {
			log.Println("Recovering panic from Kalman filter")
			err = fmt.Errorf("panic: %v", r)
			// 	log.Println("Last features:")
			// 	for i, feat := range f.DebugLastFeatures {
			// 		j, _ := json.Marshal(feat)
			// 		log.Println("obs", string(j))
			//
			// 		if i < len(f.DebugLastEstimates) {
			// 			k, _ := json.Marshal(f.DebugLastEstimates[i])
			// 			log.Println("est", string(k))
			// 		}
			// 	}
			// } else {
			f.DebugLastEstimates = append(f.DebugLastEstimates, estimatedFeature)
			if len(f.DebugLastEstimates) > 60 {
				f.DebugLastEstimates = f.DebugLastEstimates[1:]
			}
		}
	}()
	t0, t1 := mustGetTime(f.LastFeature, "Time"), mustGetTime(obs, "Time")
	defer func() {
		f.LastFeature = obs
	}()

	if t0.Equal(t1) {
		return obs, nil
	}
	if t1.Before(t0) {
		if err := f.InitFromPoint(obs); err != nil {
			return nil, fmt.Errorf("initialization error: %w", err)
		}
		return obs, fmt.Errorf("observation time is before last observation time: last=%s current=%s", t0.Format(time.RFC3339), t1.Format(time.RFC3339))
	}
	timeDelta := t1.Sub(t0)
	if timeDelta.Seconds() < 1 {
		return obs, nil
	}
	if timeDelta > 10*time.Minute {
		if err := f.InitFromPoint(obs); err != nil {
			return nil, fmt.Errorf("initialization error: %w", err)
		}
		return obs, fmt.Errorf("observation time is too far in the future: last=%s current=%s", t0.Format(time.RFC3339), t1.Format(time.RFC3339))
	}

	o := &rkalman.GeoObserved{
		Lat:                obs.Point().Lat(),
		Lng:                obs.Point().Lon(),
		Altitude:           obs.Properties["Elevation"].(float64),
		Speed:              math.Max(0, obs.Properties["Speed"].(float64)),
		SpeedAccuracy:      math.Sqrt(math.Max(2, obs.Properties["Speed"].(float64))),
		Direction:          obs.Properties["Heading"].(float64),
		DirectionAccuracy:  60,
		HorizontalAccuracy: obs.Properties["Accuracy"].(float64) + 1,
		VerticalAccuracy:   obs.Properties["Accuracy"].(float64) + 1,
	}
	err = f.Filter.Observe(timeDelta.Seconds(), o)
	if err != nil {
		return nil, fmt.Errorf("observation error: %w", err)
	}

	estimatedFeature = &geojson.Feature{} // necessary? or inited in sig?

	// The estimate value is copied from the observation.
	*estimatedFeature = *obs

	// The geometry alone is updated with the estimate.
	filterEstimate := f.Filter.Estimate()
	estimatedFeature.Geometry = orb.Point{filterEstimate.Lng, filterEstimate.Lat}
	estimatedFeature.Properties["Elevation"] = filterEstimate.Altitude
	estimatedFeature.Properties["Speed"] = filterEstimate.Speed
	estimatedFeature.Properties["Heading"] = filterEstimate.Direction
	estimatedFeature.Properties["Accuracy"] = filterEstimate.HorizontalAccuracy

	return estimatedFeature, nil
}

var errCoordinateOutOfRange = errors.New("coordinate out of range")

func validatePointFeature(f *geojson.Feature) error {
	if pt, ok := f.Geometry.(orb.Point); !ok {
		return errors.New("not a point")
	} else if pt.Lon() < -180 || pt.Lon() > 180 || pt.Lon() == 0 {
		j, _ := json.MarshalIndent(f, "", "  ")
		return fmt.Errorf("%w: longitude out of range: %f %s", errCoordinateOutOfRange, pt.Lon(), string(j))
	} else if pt.Lat() < -90 || pt.Lat() > 90 || pt.Lat() == 0 {
		j, _ := json.MarshalIndent(f, "", "  ")
		return fmt.Errorf("%w: latitude out of range: %f %s", errCoordinateOutOfRange, pt.Lon(), string(j))
	}

	if _, ok := f.Properties["Time"]; !ok {
		return errors.New("missing time")
	}
	t := mustGetTime(f, "Time")
	if t.IsZero() {
		return errors.New("invalid time")
	}
	if t.Before(time.Now().Add(-1 * time.Hour * 24 * 365 * 25)) {
		return errors.New("time is too far in the past")
	}
	if _, ok := f.Properties["UUID"]; !ok {
		return errors.New("missing UUID")
	}
	if _, ok := f.Properties["Name"]; !ok {
		return errors.New("missing name")
	}
	if _, ok := f.Properties["Accuracy"]; !ok {
		return errors.New("missing accuracy")
	}
	if _, ok := f.Properties["Elevation"]; !ok {
		return errors.New("missing elevation")
	}
	if _, ok := f.Properties["Speed"]; !ok {
		return errors.New("missing speed")
	}
	if _, ok := f.Properties["Heading"]; !ok {
		return errors.New("missing heading")
	}
	return nil
}

func readStreamWithFeatureCallback(reader io.Reader, callback func(*geojson.Feature) (*geojson.Feature, error)) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	breader := bufio.NewReaderSize(reader, 4096*256)

	go func() {
		for {
			read, err := breader.ReadBytes('\n')
			if err != nil {
				if errors.Is(err, os.ErrClosed) || errors.Is(err, io.EOF) {
					closeCh <- struct{}{}
					return
				}
				errChan <- err
			}
			pointFeature, err := geojson.UnmarshalFeature(read)
			if err != nil {
				errChan <- fmt.Errorf("failed to unmarshal geojson: %w", err)
			}
			// Nil error tolerance.
			if pointFeature == nil {
				continue
			}

			if callback != nil {
				out, err := callback(pointFeature)
				if err != nil {
					// If the callback returns an error,
					// the error will be sent to the errors channel,
					// and the next iteration will happen.
					errChan <- err
					continue
				}
				// We ONLY SEND THE FEATURE IF IT'S NOT NIL.
				featureChan <- out
			} else {
				// No callback, send the feature.
				featureChan <- pointFeature
			}
		}
	}()

	return featureChan, errChan, closeCh
}

func readStreamToLineString(reader io.Reader) (*geojson.Feature, error) {
	breader := bufio.NewReader(reader)

	ls := orb.LineString{}
	line := geojson.NewFeature(ls)

	for {
		read, err := breader.ReadBytes('\n')
		if err != nil {
			if errors.Is(err, os.ErrClosed) || errors.Is(err, io.EOF) {
				break
			}
			log.Fatalln(err)
		}

		pointFeature, err := geojson.UnmarshalFeature(read)
		if err != nil {
			log.Fatalln(err)
		}

		err = lineStringAddPoint(line, pointFeature)
		if err != nil {
			log.Fatalln(err)
		}
	}

	return line, nil
}

func mustGetTime(f *geojson.Feature, key string) time.Time {
	t := time.Time{}
	value, ok := f.Properties[key]
	if !ok {
		return t
	}
	valueStr, ok := value.(string)
	if !ok {
		return t
	}
	tt, err := time.Parse(time.RFC3339, valueStr)
	if err != nil {
		return t
	}
	return tt
}

// https://stackoverflow.com/questions/18390266/how-can-we-truncate-float64-type-to-a-particular-precision
func round(num float64) int {
	return int(num + math.Copysign(0.5, num))
}

func toFixed(num float64, precision int) float64 {
	output := math.Pow(10, float64(precision))
	return float64(round(num*output)) / output
}

type floatStats struct {
	Min  float64
	Max  float64
	Mean float64
}

func lineStringAddPoint(ls *geojson.Feature, point *geojson.Feature) error {
	_ls := ls.Geometry.(orb.LineString)
	pointFixed6 := [2]float64{toFixed(point.Point()[0], 6), toFixed(point.Point()[1], 6)}
	_ls = append(_ls, pointFixed6)
	ls.Geometry = _ls

	// Overwriting properties. 3
	if v, ok := point.Properties["Name"]; ok {
		ls.Properties["Name"] = v
	}

	if v, ok := point.Properties["UUID"]; ok {
		ls.Properties["UUID"] = v
	}

	if _, ok := ls.Properties["Accuracies"]; ok {
		ls.Properties["Accuracies"] = append(ls.Properties["Accuracies"].([]float64), point.Properties["Accuracy"].(float64))
	} else {
		ls.Properties["Accuracies"] = []float64{point.Properties["Accuracy"].(float64)}
	}

	// Bounded properties.
	t := mustGetTime(point, "Time")

	if _, ok := ls.Properties["UnixTimes"]; ok {
		ls.Properties["UnixTimes"] = append(ls.Properties["UnixTimes"].([]int64), t.Unix())
	} else {
		ls.Properties["UnixTimes"] = []int64{t.Unix()}
	}

	lsStartTime := mustGetTime(ls, "StartTime")
	if lsStartTime.IsZero() || t.Before(lsStartTime) {
		ls.Properties["StartTime"] = t.Format(time.RFC3339)
		ls.Properties["StartTimeUnix"] = t.Unix()
	}

	lsEndTime := mustGetTime(ls, "EndTime")
	if lsEndTime.IsZero() || t.After(lsEndTime) {
		ls.Properties["EndTime"] = t.Format(time.RFC3339)
		ls.Properties["EndTimeUnix"] = t.Unix()
	}

	// Aggregate properties.
	if ls.Properties["Activities"] == nil {
		ls.Properties["Activities"] = map[string]int{}
	}

	if v, ok := point.Properties["Activity"]; ok && v != "" {
		if _, ok := ls.Properties["Activities"].(map[string]int)[v.(string)]; ok {
			ls.Properties["Activities"].(map[string]int)[v.(string)]++
		} else {
			ls.Properties["Activities"].(map[string]int)[v.(string)] = 1
		}
	}

	if _, ok := ls.Properties["Elevations"]; ok {
		ls.Properties["Elevations"] = append(ls.Properties["Elevations"].([]float64), toFixed(point.Properties["Elevation"].(float64), 0))
	} else {
		ls.Properties["Elevations"] = []float64{toFixed(point.Properties["Elevation"].(float64), 0)}
	}

	if _, ok := ls.Properties["Speeds"]; ok {
		ls.Properties["Speeds"] = append(ls.Properties["Speeds"].([]float64), toFixed(point.Properties["Speed"].(float64), 1))
	} else {
		ls.Properties["Speeds"] = []float64{toFixed(point.Properties["Speed"].(float64), 1)}
	}

	if _, ok := ls.Properties["Headings"]; ok {
		ls.Properties["Headings"] = append(ls.Properties["Headings"].([]float64), toFixed(point.Properties["Heading"].(float64), 0))
	} else {
		ls.Properties["Headings"] = []float64{toFixed(point.Properties["Heading"].(float64), 0)}
	}

	min, err := stats.Min(ls.Properties["Elevations"].([]float64))
	if err != nil {
		return err
	}
	max, err := stats.Max(ls.Properties["Elevations"].([]float64))
	if err != nil {
		return err
	}
	mean, err := stats.Mean(ls.Properties["Elevations"].([]float64))
	if err != nil {
		return err
	}

	if v, ok := ls.Properties["ElevationStats"]; ok {
		v := v.(map[string]float64)
		v["Min"] = toFixed(min, 1)
		v["Max"] = toFixed(max, 1)
		v["Mean"] = toFixed(mean, 1)
		ls.Properties["ElevationStats"] = v
	} else {
		ls.Properties["ElevationStats"] = map[string]float64{
			"Min":  toFixed(min, 1),
			"Max":  toFixed(max, 1),
			"Mean": toFixed(mean, 1),
		}
	}

	min, err = stats.Min(ls.Properties["Speeds"].([]float64))
	if err != nil {
		return err
	}
	max, err = stats.Max(ls.Properties["Speeds"].([]float64))
	if err != nil {
		return err
	}
	mean, err = stats.Mean(ls.Properties["Speeds"].([]float64))
	if err != nil {
		return err
	}

	if v, ok := ls.Properties["SpeedStats"]; ok {
		v := v.(map[string]float64)
		v["Min"] = toFixed(min, 1)
		v["Max"] = toFixed(max, 1)
		v["Mean"] = toFixed(mean, 1)
		ls.Properties["SpeedStats"] = v
	} else {
		ls.Properties["SpeedStats"] = map[string]float64{
			"Min":  toFixed(min, 1),
			"Max":  toFixed(max, 1),
			"Mean": toFixed(mean, 1),
		}
	}

	return nil
}

/*
"github.com/rosshemsley/kalman"
*/
type Observation struct {
	Time     time.Time
	Point    mat.Vector
	Accuracy float64
}

func NewObservation(secondsOffset float64, x, y, accuracy float64) Observation {
	return Observation{
		Point:    mat.NewVecDense(2, []float64{x, y}),
		Time:     time.Time{}.Add(time.Duration(secondsOffset * float64(time.Second))),
		Accuracy: accuracy,
	}
}

func modifyLineStringKalman(ls *geojson.Feature) error {
	if len(ls.Geometry.(orb.LineString)) < 2 {
		return errors.New("line string must have at least two points")
	}
	observations := []Observation{}
	for i, pt := range ls.Geometry.(orb.LineString) {
		y := pt[0]
		x := pt[1]
		secondsOffset := ls.Properties["UnixTimes"].([]int64)[i]
		accuracy := ls.Properties["Accuracies"].([]float64)[i] / earthCircumferenceDegreesPerMeter
		variance := accuracy

		observations = append(observations, NewObservation(float64(secondsOffset), x, y, variance))
	}

	model := models.NewConstantVelocityModel(observations[0].Time, observations[0].Point, models.ConstantVelocityModelConfig{
		InitialVariance: observations[0].Accuracy,
		ProcessVariance: observations[0].Accuracy / 2,
	})

	filteredTrajectory, err := kalmanFilter(model, observations)
	if err != nil {
		return err
	}

	for i, fpt := range filteredTrajectory {
		ls.Geometry.(orb.LineString)[i][0] = fpt.AtVec(1) // y
		ls.Geometry.(orb.LineString)[i][1] = fpt.AtVec(0) // x
	}

	return nil
}

func kalmanFilter(model *models.ConstantVelocityModel, observations []Observation) ([]mat.Vector, error) {
	result := make([]mat.Vector, len(observations))
	filter := kalman.NewKalmanFilter(model)

	for i, obs := range observations {
		err := filter.Update(obs.Time, model.NewPositionMeasurement(obs.Point, obs.Accuracy))
		if err != nil {
			return nil, err
		}

		result[i] = model.Position(filter.State())
	}

	return result, nil
}

/*
"github.com/regnull/kalman"
*/

func modifyLineStringKalmanRegnull(ls *geojson.Feature) error {
	if len(ls.Geometry.(orb.LineString)) < 2 {
		return errors.New("line string must have at least two points")
	}

	// Estimate process noise.
	processNoise := &rkalman.GeoProcessNoise{
		// We assume the measurements will take place at the approximately the
		// same location, so that we can disregard the earth's curvature.
		BaseLat: ls.Geometry.(orb.LineString)[0][1],
		// How much do we expect the user to move, meters per second.
		DistancePerSecond: 1, //  ls.Properties["Speeds"].([]float64)[0],
		// How much do we expect the user's speed to change, meters per second squared.
		SpeedPerSecond: math.Sqrt(1),
	}
	// Initialize Kalman filter.
	filter, err := rkalman.NewGeoFilter(processNoise)
	if err != nil {
		fmt.Printf("failed to initialize Kalman filter: %s\n", err)
		os.Exit(1)
	}

	lastTimeUnix := ls.Properties["UnixTimes"].([]int64)[0] - 1

	for i, pt := range ls.Geometry.(orb.LineString) {
		x := pt[0]
		y := pt[1]
		// secondsOffset := ls.Properties["UnixTimes"].([]int64)[i]
		// accuracy := ls.Properties["Accuracies"].([]float64)[i] / earthCircumferenceDegreesPerMeter
		// variance := accuracy

		td := float64(ls.Properties["UnixTimes"].([]int64)[i]) - float64(lastTimeUnix)
		lastTimeUnix = ls.Properties["UnixTimes"].([]int64)[i]

		log.Printf("lat %f, lng %f, speed %f, accuracy %f", y, x, ls.Properties["Speeds"].([]float64)[i], ls.Properties["Accuracies"].([]float64)[i])
		obs := &rkalman.GeoObserved{
			Lat:                y,
			Lng:                x,
			Altitude:           ls.Properties["Elevations"].([]float64)[i],
			Speed:              math.Max(0, ls.Properties["Speeds"].([]float64)[i]),
			SpeedAccuracy:      math.Sqrt(math.Max(0, ls.Properties["Speeds"].([]float64)[i])),
			Direction:          ls.Properties["Headings"].([]float64)[i],
			DirectionAccuracy:  10,
			HorizontalAccuracy: ls.Properties["Accuracies"].([]float64)[i] + 1,
			VerticalAccuracy:   1,
		}

		if err := filter.Observe(td, obs); err != nil {
			return fmt.Errorf("observation error: %w", err)
		}

		estimate := filter.Estimate()

		ls.Geometry.(orb.LineString)[i][0] = estimate.Lng
		ls.Geometry.(orb.LineString)[i][1] = estimate.Lat
	}
	return nil
}
