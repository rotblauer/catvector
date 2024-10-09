package main

import (
	"time"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/planar"
)

type StopConsolidator struct {
	StopDistanceThreshold float64
	Features              TracksGeoJSON

	// StopPoint is the synthesized stop point.
	StopPoint TrackGeoJSON

	// stopPointN is the incremental counter for stop points.
	// It is used to differentiate returned stop points as the feature ID.
	// This is important because the caller wants to only flush (write)
	// the completed stop points, and to NOT flush stop points that are still being built.
	stopPointN int
}

func (sc *StopConsolidator) Reset() {
	sc.Features = TracksGeoJSON{}
	sc.StopPoint = sc.NewStopPoint()
}

func (sc *StopConsolidator) NewStopPoint() TrackGeoJSON {
	stopPoint := geojson.NewFeature(orb.Point{})
	sc.stopPointN++
	stopPoint.Properties = map[string]interface{}{
		"ID":       sc.stopPointN,
		"Time":     time.Time{},
		"Count":    0,
		"Duration": 0.0,
		"MaxDist":  0.0,
		"P50Dist":  0.0,
		"P99Dist":  0.0,
	}
	return TrackGeoJSON{stopPoint}
}

func (sc *StopConsolidator) LastFeature() *geojson.Feature {
	if len(sc.Features) == 0 {
		return nil
	}
	return sc.Features[len(sc.Features)-1].Feature
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

func (sc *StopConsolidator) MergeStopPoint(f *geojson.Feature) {
	lastFeature := sc.LastFeature()
	sc.Features = append(sc.Features, &TrackGeoJSON{f})

	sc.StopPoint.Properties["Name"] = f.Properties.MustString("Name")
	sc.StopPoint.Properties["UUID"] = f.Properties.MustString("UUID")
	sc.StopPoint.Properties["Version"] = f.Properties.MustString("Version")

	// TODO: Synthesize?
	if act, ok := f.Properties["Activity"]; ok {
		sc.StopPoint.Properties["Activity"] = act
	} else {
		sc.StopPoint.Properties["Activity"] = "Unknown"
	}
	sc.StopPoint.Properties["Speed"] = f.Properties.MustFloat64("Speed")
	sc.StopPoint.Properties["Elevation"] = f.Properties.MustFloat64("Elevation")
	sc.StopPoint.Properties["Heading"] = f.Properties.MustFloat64("Heading")
	sc.StopPoint.Properties["Accuracy"] = f.Properties.MustFloat64("Accuracy")

	if sc.StopPoint.Properties["MotionStateReason"] == nil {
		sc.StopPoint.Properties["MotionStateReason"] = f.Properties.MustString("MotionStateReason")
	}

	// Duration is the time between the first and last point in the stop.
	// The first feature time difference will == 0, subsequent features will have a duration.
	if lastFeature != nil {
		sc.StopPoint.Feature.Properties["Duration"] =
			sc.StopPoint.Feature.Properties.MustFloat64("Duration") +
				mustGetTime(f, "Time").Sub(mustGetTime(lastFeature, "Time")).
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
	points := []orb.Point{}
	for _, f := range sc.Features {
		points = append(points, f.Point())
	}
	mp := orb.MultiPoint(points)
	centroid, _ := planar.CentroidArea(mp)
	sc.StopPoint.Geometry = centroid

	distances := []float64{}
	for _, f := range sc.Features {
		distances = append(distances, geo.Distance(centroid, f.Point()))
	}
	distP50, _ := stats.Percentile(distances, 50)
	distP99, _ := stats.Percentile(distances, 99)
	sc.StopPoint.Feature.Properties["P50Dist"] = distP50
	sc.StopPoint.Feature.Properties["P99Dist"] = distP99

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
	delta := mustGetTime(f, "Time").Sub(mustGetTime(sc.LastFeature(), "Time"))
	if delta.Seconds() < -1 {
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
