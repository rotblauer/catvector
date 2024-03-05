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
	"errors"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"time"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
	rkalman "github.com/regnull/kalman"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

const (
	earthRadius                       = 6378137.0 // meters
	earthCircumference                = math.Pi * earthRadius * 2
	earthCircumferenceMetersPerDegree = earthCircumference / 360
	earthCircumferenceDegreesPerMeter = 360 / earthCircumference
)

func main() {
	lineStringFeature, err := readStreamToLineString(os.Stdin)
	if err != nil {
		log.Fatalln(err)
	}

	bwriter := bufio.NewWriter(os.Stdout)
	j, err := lineStringFeature.MarshalJSON()
	if err != nil {
		log.Fatalln(err)
	}
	if _, err := bwriter.Write(j); err != nil {
		log.Fatalln(err)
	}
	if err := bwriter.Flush(); err != nil {
		log.Fatalln(err)
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

func (f *RKalmanFilterT) EstimateFromObservation(obs *geojson.Feature) (estimate *geojson.Feature, err error) {
	if obs.Properties["Time"].(time.Time).Equal(f.LastFeature.Properties["Time"].(time.Time)) {
		return obs, nil
	}

	timeDelta := obs.Properties["Time"].(time.Time).Sub(f.LastFeature.Properties["Time"].(time.Time))
	if timeDelta.Seconds() < 0 {
		return nil, fmt.Errorf("observation time is before last observation time")
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

	estimate = &geojson.Feature{} // necessary? or inited in sig?
	*estimate = *obs
	guess := f.Filter.Estimate()
	estimate.Geometry = orb.Point{guess.Lng, guess.Lat}
	estimate.Properties["Elevation"] = guess.Altitude
	estimate.Properties["Speed"] = guess.Speed
	estimate.Properties["Heading"] = guess.Direction
	estimate.Properties["Accuracy"] = guess.HorizontalAccuracy

	f.LastFeature = obs
	return estimate, nil
}

func readStreamRKalmanFilter(reader io.Reader) (chan *geojson.Feature, chan error) {
	featureChan := make(chan *geojson.Feature)
	errChan := make(chan error)

	filter := &RKalmanFilterT{}
	breader := bufio.NewReader(reader)

	go func() {
		for {
			read, err := breader.ReadBytes('\n')
			if err != nil {
				if errors.Is(err, os.ErrClosed) || errors.Is(err, io.EOF) {
					return
				}
				log.Fatalln(err)
			}
			pointFeature, err := geojson.UnmarshalFeature(read)
			if err != nil {
				log.Fatalln(err)
			}

			if filter.Filter == nil {
				filter.InitFromPoint(pointFeature)
			}

			estimate, err := filter.EstimateFromObservation(pointFeature)
			if err != nil {
				errChan <- err
				continue
			}
			featureChan <- estimate
		}
	}()

	return featureChan, errChan
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
	if _, ok := f.Properties[key]; !ok {
		return t
	}
	tt, err := time.Parse(time.RFC3339, f.Properties[key].(string))
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
