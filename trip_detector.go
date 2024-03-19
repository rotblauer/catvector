package main

import (
	"math"
	"time"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
)

var (
	speedThreshold = 0.5
)

// TripDetector normally takes Points and turns them into either a Trip or a Stop.
// It is a state machine that takes in a series of points and returns a series of Trips and Stops.
// Trips are defined as a series of points that are moving, structured as LineStrings.
// Stops are defined as a series of points that are stationary, structured as Points.
type TripDetector struct {
	Interval       time.Duration
	SpeedThreshold float64
	lastNPoints    TracksGeoJSON
	intervalPoints TracksGeoJSON
	Tripping       bool
}

func NewTripDetector(interval time.Duration, speedThreshold float64) *TripDetector {
	return &TripDetector{
		Interval:       interval,
		SpeedThreshold: speedThreshold,
		Tripping:       false,
		lastNPoints:    TracksGeoJSON{},
		intervalPoints: TracksGeoJSON{},
	}
}

func (d *TripDetector) LastPoint() *TrackGeoJSON {
	if len(d.lastNPoints) == 0 {
		return nil
	}
	return d.lastNPoints[len(d.lastNPoints)-1]
}

func (d *TripDetector) AddFeatureToState(f *geojson.Feature) {
	d.intervalPoints = append(d.intervalPoints, &TrackGeoJSON{f})
	for i := len(d.intervalPoints) - 1; i > 0; i-- {
		if d.intervalPoints.Timespan() > d.Interval {
			d.intervalPoints = d.intervalPoints[i:]
			break
		}
	}

	d.lastNPoints = append(d.lastNPoints, &TrackGeoJSON{f})
	for len(d.lastNPoints) > 10 {
		d.lastNPoints = d.lastNPoints[1:]
	}
}

func (d *TripDetector) ResetState() {
	d.lastNPoints = TracksGeoJSON{}
	d.intervalPoints = TracksGeoJSON{}
	d.Tripping = false
}

func (d *TripDetector) AddFeature(f *geojson.Feature) error {
	t := &TrackGeoJSON{f}

	if last := d.LastPoint(); last != nil {
		if last.MustGetTime().After(t.MustGetTime()) {
			d.ResetState()
		}
	}
	d.AddFeatureToState(f)

	// Get the average reported speed from all the points in the interval.
	// Get the average geo-calculated speed from all the points in the interval.
	// Weight the reported speed 80% and the calculated speed 20%.
	referenceSpeeds := d.intervalPoints.ReportedSpeeds()
	calculatedSpeeds := d.intervalPoints.CalculatedSpeeds()
	for i, s := range calculatedSpeeds {
		referenceSpeeds[i] = referenceSpeeds[i]*0.5 + s*0.5
	}

	// Get the statistics for this range.
	referenceStats := stats.Float64Data(referenceSpeeds)

	mean, err := referenceStats.Mean()
	if err != nil {
		return err
	}
	if mean > d.SpeedThreshold*2 {
		d.Tripping = true
		return nil
	}

	median, err := referenceStats.Median()
	if err != nil {
		return err
	}
	if median > d.SpeedThreshold {
		d.Tripping = true
		return nil
	}

	step := 0.5 // ~ 1 mile/hour
	steppedReferenceSpeeds := []float64{}
	for _, s := range referenceSpeeds {
		if s < step {
			steppedReferenceSpeeds = append(steppedReferenceSpeeds, 0)
			continue
		}
		rem := math.Mod(s, step)
		stepped := s - rem
		steppedReferenceSpeeds = append(steppedReferenceSpeeds, stepped)
	}
	mode, err := stats.Float64Data(steppedReferenceSpeeds).Mode()
	if err != nil {
		return err
	}
	if len(mode) > 0 {
		if mode[0] > d.SpeedThreshold {
			d.Tripping = true
			return nil
		} else if mode[0] == 0 {
			d.Tripping = false
			return nil
		}
	}

	// accuracies := []float64{}
	// d.intervalPoints.ForEach(func(t *TrackGeoJSON) {
	// 	accuracies = append(accuracies, t.MustGetAccuracy())
	// })
	// accuracyStats := stats.Float64Data(accuracies)
	// averageAccuracy, err := accuracyStats.Mean()
	// if err != nil {
	// 	return err
	// }

	// // Compute the center of these points.
	// // If the distance from the center of all the points is less than the average accuracy*2, we are stationary.
	// mls := geojson.NewFeature(orb.MultiPoint(d.intervalPoints.Points()))
	// centroid, _ := planar.CentroidArea(mls.Geometry.(orb.MultiPoint))
	// // area = math.Abs(area)
	//
	// // If the distance from the center of all the points is less than the average accuracy*2, we are stationary.
	// outlier := false
	// for _, p := range d.intervalPoints {
	// 	if geo.Distance(p.Point(), centroid) > averageAccuracy*2 {
	// 		outlier = true
	// 	}
	// }
	// if !outlier {
	// 	d.Tripping = false
	// 	return nil
	// }

	// Still in region?
	if d.intervalPoints.Timespan() > d.Interval/2 {
		firstPointFromInterval := d.intervalPoints[0]
		radius := d.SpeedThreshold * d.Interval.Seconds()
		if geo.Distance(firstPointFromInterval.Point(), t.Point()) < radius {
			d.Tripping = false
			return nil
		}
	}

	// NOOP: If we are still here, we are UNDECIDED.
	// The TripDetector maintains its state invariantly.

	return nil
}
