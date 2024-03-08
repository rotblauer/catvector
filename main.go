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
	"regexp"
	"time"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
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

func cmdRKalmanFilter() {
	bwriter := bufio.NewWriter(os.Stdout)
	featureCh, errCh, closeCh := readStreamRKalmanFilter(os.Stdin)

loop:
	for {
		select {
		case feature := <-featureCh:
			j, err := feature.MarshalJSON()
			if err != nil {
				log.Fatalln(err)
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

type TrackerStateActivity int

const (
	TrackerStateUnknown TrackerStateActivity = iota
	TrackerStateStationary
	TrackerStateWalking
	TrackerStateRunning
	TrackerStateCycling
	TrackerStateDriving
)

var (
	activityStationary = regexp.MustCompile(`(?i)stationary|still`)
	activityWalking    = regexp.MustCompile(`(?i)walk`)
	activityRunning    = regexp.MustCompile(`(?i)run`)
	activityCycling    = regexp.MustCompile(`(?i)cycle|bike|biking`)
	activityDriving    = regexp.MustCompile(`(?i)drive|driving|automotive`)
)

func (a TrackerStateActivity) IsMoving() bool {
	return a > TrackerStateStationary
}

func TrackerStateActivityFromReport(report interface{}) TrackerStateActivity {
	if report == nil {
		return TrackerStateUnknown
	}
	reportStr, ok := report.(string)
	if !ok {
		return TrackerStateUnknown
	}
	switch {
	case activityStationary.MatchString(reportStr):
		return TrackerStateStationary
	case activityWalking.MatchString(reportStr):
		return TrackerStateWalking
	case activityRunning.MatchString(reportStr):
		return TrackerStateRunning
	case activityCycling.MatchString(reportStr):
		return TrackerStateCycling
	case activityDriving.MatchString(reportStr):
		return TrackerStateDriving
	}
	return TrackerStateUnknown
}

type TrackerState struct {
	AverageSpeed float64
	IsMoving     bool
	Activity     TrackerStateActivity
}

type Tracker struct {
	Interval           time.Duration
	State              TrackerState
	intervalFeatures   []*geojson.Feature // points
	LineStringFeatures []*geojson.Feature // linestrings
}

func NewTracker(interval time.Duration) *Tracker {
	return &Tracker{
		Interval: interval,
		State: TrackerState{
			Activity: TrackerStateUnknown,
		},
		intervalFeatures:   make([]*geojson.Feature, 0),
		LineStringFeatures: make([]*geojson.Feature, 0),
	}
}

// calculatedAverageSpeedAbsolute returns the average speed in meters per second
// between the first point and last. Round trips will theoretically return 0.
func calculatedAverageSpeedAbsolute(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return 0
	}
	firstTime := mustGetTime(pointFeatures[0], "Time")
	lastTime := mustGetTime(pointFeatures[len(pointFeatures)-1], "Time")
	timeDelta := lastTime.Sub(firstTime).Seconds()
	distance := geo.Distance(pointFeatures[0].Point(), pointFeatures[len(pointFeatures)-1].Point())

	return distance / timeDelta
}

func averageReportedSpeed(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return pointFeatures[0].Properties["Speed"].(float64)
	}
	sum := 0.0
	for _, f := range pointFeatures {
		sum += f.Properties["Speed"].(float64)
	}
	return sum / float64(len(pointFeatures))
}

func timespan(pointFeatures []*geojson.Feature) time.Duration {
	if len(pointFeatures) < 2 {
		return 0
	}
	firstTime := mustGetTime(pointFeatures[0], "Time")
	lastTime := mustGetTime(pointFeatures[len(pointFeatures)-1], "Time")
	return lastTime.Sub(firstTime)
}

func distanceAbsolute(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return 0
	}
	return geo.Distance(pointFeatures[0].Point(), pointFeatures[len(pointFeatures)-1].Point())
}

func (t *Tracker) LastLinestring() *geojson.Feature {
	if len(t.LineStringFeatures) == 0 {
		return nil
	}
	return t.LineStringFeatures[len(t.LineStringFeatures)-1]
}

func (t *Tracker) AddPointFeatureToLastLinestring(f *geojson.Feature) {
	ls := t.LineStringFeatures[len(t.LineStringFeatures)-1]
	ls.Geometry = append(ls.Geometry.(orb.LineString), f.Point())
	// TODO: update properties
	ls.Properties["Duration"] = mustGetTime(f, "Time").Sub(mustGetTime(ls, "StartTime")).Round(time.Second).Seconds()
	for k, v := range f.Properties {
		ls.Properties[k] = v
	}
	t.LineStringFeatures[len(t.LineStringFeatures)-1] = ls
}

func (t *Tracker) AddPointFeatureToNewLinestring(f *geojson.Feature) {
	newLineString := geojson.NewFeature(orb.LineString{f.Point()})
	newLineString.Properties = f.Properties
	newLineString.Properties["StartTime"] = f.Properties["Time"]
	newLineString.Properties["Duration"] = 0.0
	newLineString.Properties["IsMoving"] = t.State.IsMoving

	t.LineStringFeatures = append(t.LineStringFeatures, newLineString)
}

// isDiscontinuous returns true if the last point is discontinuous from the previous interval.
func (t *Tracker) isDiscontinuous() (isDiscontinuous bool) {
	f := t.intervalFeatures[len(t.intervalFeatures)-1]
	if f.Properties["UUID"] != t.LastLinestring().Properties["UUID"] {
		return true
	}
	if f.Properties["Activity"] != t.LastLinestring().Properties["Activity"] {
		return true
	}

	// This
	dist := distanceAbsolute(t.intervalFeatures)
	// 0.8 here becomes m/s -> 1.11847 mph
	// METERS > 0.8 * 30s = 24m
	intervalIsMoving := dist > 0.8*flagTrackerInterval.Seconds()
	if intervalIsMoving != t.State.IsMoving {
		return true
	}

	return false
}

func (t *Tracker) AddPointFeature(f *geojson.Feature) {

	if len(t.LineStringFeatures) == 0 {
		t.intervalFeatures = append(t.intervalFeatures, f)
		t.AddPointFeatureToNewLinestring(f)
		return
	}

	t.intervalFeatures = append(t.intervalFeatures, f)
	for timespan(t.intervalFeatures) > t.Interval {
		t.intervalFeatures = t.intervalFeatures[1:]
	}

	if t.isDiscontinuous() {
		t.State.Activity = TrackerStateActivityFromReport(f.Properties["Activity"])
		t.State.IsMoving = t.State.Activity.IsMoving()
		t.AddPointFeatureToNewLinestring(f)
	} else {
		t.AddPointFeatureToLastLinestring(f)
	}
}

var flagTrackerInterval = flag.Duration("interval", 10*time.Second, "line detection interval")

func cmdPointsToLineStrings() {
	t := NewTracker(*flagTrackerInterval)
	featureCh, errCh, closeCh := readStreamWithFeatureCallback(os.Stdin, func(f *geojson.Feature) (*geojson.Feature, error) {
		return f, nil
	})
loop:
	for {
		select {
		case f := <-featureCh:
			t.AddPointFeature(f)
		case err := <-errCh:
			log.Println(err)
		case <-closeCh:
			break loop
		}
	}
	for _, ls := range t.LineStringFeatures {
		j, err := ls.MarshalJSON()
		if err != nil {
			log.Fatalln(err)
		}
		j = append(j, []byte("\n")...)
		if _, err := os.Stdout.Write(j); err != nil {
			log.Fatalln(err)
		}
	}
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
	case "linestrings-to-points":
		cmdLinestringsToPoints()
		return
	case "douglas-peucker":
		cmdDouglasPeucker()
		return
	default:
		log.Fatalf("unknown command: %s", command)
	}
}

type RKalmanFilterT struct {
	ProcessNoise *rkalman.GeoProcessNoise
	Filter       *rkalman.GeoFilter
	LastFeature  *geojson.Feature
}

func (f *RKalmanFilterT) InitFromPoint(obs *geojson.Feature) error {
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
	t0, t1 := mustGetTime(f.LastFeature, "Time"), mustGetTime(obs, "Time")
	defer func() {
		f.LastFeature = obs
	}()

	if t0.Equal(t1) {
		return obs, nil
	}
	if t1.Before(t0) {
		return obs, fmt.Errorf("observation time is before last observation time: last=%s current=%s", t0.Format(time.RFC3339), t1.Format(time.RFC3339))

	}
	timeDelta := t1.Sub(t0)
	if timeDelta.Seconds() < 1 {
		return obs, nil
	}

	o := &rkalman.GeoObserved{
		Lat:                obs.Point().Lat(),
		Lng:                obs.Point().Lon(),
		Altitude:           obs.Properties["Elevation"].(float64),
		Speed:              math.Max(0, obs.Properties["Speed"].(float64)),
		SpeedAccuracy:      math.Sqrt(math.Max(0, obs.Properties["Speed"].(float64))),
		Direction:          obs.Properties["Heading"].(float64),
		DirectionAccuracy:  10,
		HorizontalAccuracy: obs.Properties["Accuracy"].(float64) + 1,
		VerticalAccuracy:   1,
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
	// estimatedFeature.Properties["Elevation"] = filterEstimate.Altitude
	// estimatedFeature.Properties["Speed"] = filterEstimate.Speed
	// estimatedFeature.Properties["Heading"] = filterEstimate.Direction
	// estimatedFeature.Properties["Accuracy"] = filterEstimate.HorizontalAccuracy

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

func readStreamRKalmanFilter(reader io.Reader) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	uuidFilters := map[string]*RKalmanFilterT{}
	breader := bufio.NewReader(reader)

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
				errChan <- err
			}

			filter, ok := uuidFilters[pointFeature.Properties["UUID"].(string)]
			if !ok {
				filter = &RKalmanFilterT{}
				if err := filter.InitFromPoint(pointFeature); err != nil {
					errChan <- err
					continue
				}
				// Otherwise, we've just inited the Kalman filter for this UUID.
				// Since its the filter's initial point, there's no sense in estimating from one point.
				uuidFilters[pointFeature.Properties["UUID"].(string)] = filter
				featureChan <- pointFeature
				continue
			}

			estimate, err := filter.EstimateFromObservation(pointFeature)
			if err != nil {
				errChan <- err
				continue
			}
			featureChan <- estimate
		}
	}()

	return featureChan, errChan, closeCh
}

func readStreamWithFeatureCallback(reader io.Reader, callback func(*geojson.Feature) (*geojson.Feature, error)) (chan *geojson.Feature, chan error, chan struct{}) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)
	closeCh := make(chan struct{}, 1)

	breader := bufio.NewReader(reader)

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
				errChan <- err
			}
			if pointFeature == nil {
				continue
			}

			if callback != nil {
				out, err := callback(pointFeature)
				if err != nil {
					errChan <- err
					continue
				}
				featureChan <- out
			} else {
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
