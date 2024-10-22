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
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"runtime/pprof"
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

func cmdValidate(i io.ReadCloser, o io.WriteCloser) {
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, func(f *geojson.Feature) (*geojson.Feature, error) {
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
			if _, err := o.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
}

var flagDwellInterval = flag.Duration("dwell-interval", 10*time.Second, "stop detection interval")
var flagDwellDistanceThresholdDefault = 10.0
var flagDwellDistanceThreshold = flag.Float64("dwell-distance", flagDwellDistanceThresholdDefault, "cluster distance threshold for trip stops")
var flagTrackerSpeedThreshold = flag.Float64("speed-threshold", 0.5, "speed threshold for trip detection")

func cmdPointsToLineStrings(i io.ReadCloser, o io.WriteCloser) {
	trackerWaiter := sync.WaitGroup{}
	uuidTrackers := map[string]*LineStringBuilder{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, nil)

	writeMu := sync.Mutex{}
	flushPointLocking := func(f *geojson.Feature) {
		j, err := f.MarshalJSON()
		if err != nil {
			log.Println(spew.Sdump(f))
			log.Fatalln(err)
		}
		j = append(j, []byte("\n")...)
		writeMu.Lock()
		if _, err := o.Write(j); err != nil {
			log.Fatalln(err)
		}
		writeMu.Unlock()
	}
loop:
	for {
		select {
		case f := <-featureCh:
			tracker, ok := uuidTrackers[f.Properties["UUID"].(string)]
			if !ok {
				tracker = NewLineStringBuilder(*flagDwellInterval)
				uuidTrackers[f.Properties["UUID"].(string)] = tracker
				// The write operation should be held by a mutex or the streams should be independent,
				// otherwise the async go routines can interrupt each other's lines.
				go func() {
					trackerWaiter.Add(1)
					defer trackerWaiter.Done()
					for ls := range tracker.linestringsCh {
						flushPointLocking(ls)
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

var flagDouglasPeuckerThreshold = flag.Float64("threshold", 0.0001, "Douglas-Peucker epsilon threshold")

func featureMustBeLinestring(f *geojson.Feature) (*geojson.Feature, error) {
	if _, ok := f.Geometry.(orb.LineString); !ok {
		return nil, errors.New("not a linestring")
	}
	return f, nil
}

func cmdDouglasPeucker(i io.ReadCloser, o io.WriteCloser) {
	log.Println("Douglas-Peucker simplification with threshold", *flagDouglasPeuckerThreshold)
	simplifier := simplify.DouglasPeucker(*flagDouglasPeuckerThreshold)
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, featureMustBeLinestring)
loop:
	for {
		select {
		case f := <-featureCh:
			if f == nil {
				continue
			}
			ls := geojson.LineString(f.Geometry.(orb.LineString))
			simplerGeometry := simplifier.Simplify(ls.Geometry())
			f.Geometry = simplerGeometry
			j, err := f.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := o.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
}

func cmdTripDetector(i io.ReadCloser, o io.WriteCloser) {

	if *cpuprofile != "" {
		f, err := os.Create(*cpuprofile)
		if err != nil {
			log.Fatal(err)
		}
		pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}

	uuidTripDetectors := map[string]*TripDetector{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, nil)
loop:
	for {
		select {
		case f := <-featureCh:
			if f == nil {
				panic("nil feature")
			}
			td, ok := uuidTripDetectors[f.Properties.MustString("UUID")]
			if !ok {
				td = NewTripDetector(*flagDwellInterval, *flagTrackerSpeedThreshold, *flagDwellDistanceThreshold)
				uuidTripDetectors[f.Properties.MustString("UUID")] = td
			}
			if err := td.AddFeature(f); err != nil {
				log.Fatalln(err)
			}
			f.Properties["IsTrip"] = td.Tripping
			f.Properties["MotionStateReason"] = td.MotionStateReason

			j, err := json.Marshal(f)
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := o.Write(j); err != nil {
				log.Fatalln(err)
			}
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

var flagUrbanCanyonDistance = flag.Float64("urban-canyon-distance", 200.0, "urban canyon distance threshold")

// wangUrbanCanyonFilterStream removes spurious GPS readings caused by the urban canyon effect.
// > Wang: Third, GPS points away from
// the adjacent points due to the signal shift caused by
// blocking or ‘‘urban canyon’’ effect are also deleted. As
// is shown in Figure 2, GPS points away from both the
// before and after 5 points center for more than 200 m
// should be considered as shift points.
func wangUrbanCanyonFilterStream(featuresCh chan *geojson.Feature, closingCh chan struct{}) (chan *geojson.Feature, chan error, chan struct{}) {
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
				if geo.Distance(tailCenter, target.Point()) > spuriousDistanceMeters && geo.Distance(headCenter, target.Point()) > spuriousDistanceMeters {
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
					if len(buffer) > 6 {
						for _, f := range buffer[6:] {
							featureChan <- f
						}
					}
				}
				close(featureChan)
				closeCh <- x
				return
			}
		}
	}()
	return featureChan, errChan, closeCh
}

// Declare a teleportFactor float64 flag.
var flagTeleportFactor = flag.Float64("teleport-factor", 10.0, "teleportation factor")
var flagTeleportIntervalMax = flag.Duration("teleport-interval-max", 2*time.Minute, "teleportation interval max")

func teleportationFilterStream(featuresCh chan *geojson.Feature, closingCh chan struct{}) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	lastFeature := &geojson.Feature{Geometry: orb.Point{0, 0}}

	handleFeature := func(f *geojson.Feature) {
		if f == nil {
			return
		}
		if lastFeature.Point().Lat() == 0 && lastFeature.Point().Lon() == 0 {
			lastFeature = f
			featureChan <- f
			return
		}
		dist := geo.Distance(lastFeature.Point(), f.Point())
		span := mustGetTime(f, "Time").Sub(mustGetTime(lastFeature, "Time"))

		// If we exceed the teleportation interval max, it's a reset because the cat has now
		// been wandering too long untracked.
		if span > *flagTeleportIntervalMax {
			lastFeature = f
			featureChan <- f
			return
		}

		// Compare the reported speed against the calculated speed.
		// If the calculated speed exceeds the reported speed by X factor, it's a teleportation point.
		calculatedSpeed := dist / span.Seconds()
		reportedSpeed := f.Properties.MustFloat64("Speed")
		if calculatedSpeed > reportedSpeed*(*flagTeleportFactor) {
			errChan <- fmt.Errorf("teleportation point: %v", f.Properties["Time"])
			return
		}

		// Else the last and cursor points passed the teleportation challenge!
		// They are near enough in time and space.
		lastFeature = f
		featureChan <- f
	}

	go func() {
		for {
			select {
			case f := <-featuresCh:
				handleFeature(f)
			case x := <-closingCh:
				// Drain the feature chan.
				for f := range featuresCh {
					handleFeature(f)
				}
				close(featureChan)
				closeCh <- x
				return
			}
		}
	}()
	return featureChan, errChan, closeCh
}

func cmdPreprocess(i io.ReadCloser, o io.WriteCloser) {
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, preProcessFilters)
	wangProcessedCh, wangProcErrCh, wangDoneCh := wangUrbanCanyonFilterStream(featureCh, closeCh)
	teleProcessedCh, teleProcErrCh, teleDoneCh := teleportationFilterStream(wangProcessedCh, wangDoneCh)

	okFeatures := 0
	wangUrbanCanyonHits, teleportationHits := 0, 0

	defer func() {
		// Log the ratios of wangUrbanCanyonHits and teleportationHits to okFeatures.
		totalFeatures := okFeatures + wangUrbanCanyonHits + teleportationHits
		log.Printf("PREPROCESS: wangUrbanCanyonHits: %d (%0.1f%%), teleportationHits: %d (%0.1f%%), okFeatures: %d (%0.1f%%)\n",
			wangUrbanCanyonHits, (float64(wangUrbanCanyonHits)/float64(totalFeatures))*100.0,
			teleportationHits, (float64(teleportationHits)/float64(totalFeatures))*100.0,
			okFeatures, (float64(okFeatures)/float64(totalFeatures))*100.0)
	}()

loop:
	for {
		select {
		case f := <-teleProcessedCh:
			if f == nil {
				continue
			}
			okFeatures++
			j, err := f.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
			}
			j = append(j, []byte("\n")...)
			if _, err := o.Write(j); err != nil {
				log.Fatalln(err)
			}
		case err := <-errCh:
			log.Println(err)
		case <-wangProcErrCh:
			wangUrbanCanyonHits++
			//log.Println(err)

		case <-teleProcErrCh:
			teleportationHits++
			//log.Println(err)
		case <-teleDoneCh:
			break loop
		}
	}
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
func cmdConsolidateStops(i io.ReadCloser, o io.WriteCloser) {
	uuidTrackers := map[string]*StopConsolidator{}
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, nil)
	lastStop := TrackGeoJSON{}
	flushPoint := func(feature *geojson.Feature) {
		j, err := feature.MarshalJSON()
		if err != nil {
			log.Fatalln(err, spew.Sdump(feature))
		}
		j = append(j, []byte("\n")...)
		if _, err := o.Write(j); err != nil {
			log.Fatalln(err)
		}
	}
loop:
	for {
		select {
		case f := <-featureCh:
			tracker, ok := uuidTrackers[f.Properties["UUID"].(string)]
			if !ok {
				tracker = NewStopConsolidator(*flagDwellDistanceThreshold)
				uuidTrackers[f.Properties["UUID"].(string)] = tracker
			}
			stopPoint := tracker.AddFeature(f)
			if lastStop.Feature == nil {
				lastStop = stopPoint
			}

			// If the last stop is not the same as the current stop, flush the last stop.
			// The StopConsolidator has started to build a new stop.
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

//// cmdFilterBoundingBox is an experiment to filter out linestrings which are busy rats' nests
//// inside of small bounding boxes. The idea is to filter out linestrings which are too dense.
//// The bounding box is calculated from the linestring's bound.
//// If the bounding box is smaller than a certain threshold, the linestring is filtered out.
//func cmdFilterBoundingBox(i io.ReadCloser, o io.WriteCloser) {
//	featureCh, errCh, closeCh := readStreamWithFeatureCallback(i, nil)
//loop:
//	for {
//		select {
//		case f := <-featureCh:
//			if f == nil {
//				continue
//			}
//			if ln, ok := f.Geometry.(orb.LineString); ok {
//				bound := ln.Bound()
//
//			}
//			j, err := f.MarshalJSON()
//			if err != nil {
//				log.Fatalln(err)
//			}
//			j = append(j, []byte("\n")...)
//			if _, err := o.Write(j); err != nil {
//				log.Fatalln(err)
//			}
//		case err := <-errCh:
//			log.Println(err)
//		case <-closeCh:
//			break loop
//		}
//	}
//}

var cpuprofile = flag.String("cpuprofile", "", "write cpu profile to file")

func main() {
	flag.Parse()

	command := flag.Arg(0)
	switch command {
	case "validate":
		cmdValidate(os.Stdin, os.Stdout)
		return
	case "preprocess":
		cmdPreprocess(os.Stdin, os.Stdout)
		return
	case "trip-detector":
		cmdTripDetector(os.Stdin, os.Stdout)
		return
	case "points-to-linestrings":
		cmdPointsToLineStrings(os.Stdin, os.Stdout)
		return
	case "douglas-peucker":
		cmdDouglasPeucker(os.Stdin, os.Stdout)
		return
	case "consolidate-stops":
		cmdConsolidateStops(os.Stdin, os.Stdout)
		return
	// case "rkalman":
	// 	cmdRKalmanFilter()
	// 	return
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

func readStreamWithFeatureCallback(reader io.Reader, callback func(*geojson.Feature) (*geojson.Feature, error)) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	breader := bufio.NewReaderSize(reader, 4096*512)

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
				if out != nil {
					featureChan <- out
				}
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
