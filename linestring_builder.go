package main

import (
	"flag"
	"log"
	"math"
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
)

type LineStringBuilder struct {
	Interval           time.Duration
	LineStringFeature  *geojson.Feature
	LineStringFeatures []*geojson.Feature // the points represented by the linestring
	lastNFeatures      []*geojson.Feature
	intervalFeatures   []*geojson.Feature // points
	linestringsCh      chan *geojson.Feature
}

func NewLineStringBuilder(interval time.Duration) *LineStringBuilder {
	return &LineStringBuilder{
		Interval:         interval,
		lastNFeatures:    make([]*geojson.Feature, 0),
		intervalFeatures: make([]*geojson.Feature, 0),
		linestringsCh:    make(chan *geojson.Feature),
	}
}

func (t *LineStringBuilder) LastFeature() *geojson.Feature {
	if len(t.lastNFeatures) == 0 {
		return nil
	}
	return t.lastNFeatures[len(t.lastNFeatures)-1]
}

func (t *LineStringBuilder) Flush() {
	if t.LineStringFeature != nil {
		t.linestringsCh <- t.LineStringFeature
	}
	close(t.linestringsCh)
}

func (t *LineStringBuilder) AddFeatureToState(f *geojson.Feature) {
	t.intervalFeatures = append(t.intervalFeatures, f)
	for i := len(t.intervalFeatures) - 1; i > 0; i-- {
		if timespan(t.intervalFeatures[i], f) > t.Interval {
			t.intervalFeatures = t.intervalFeatures[i:]
			break
		}
	}

	t.lastNFeatures = append(t.lastNFeatures, f)
	for len(t.lastNFeatures) > 10 {
		t.lastNFeatures = t.lastNFeatures[1:]
	}
}

func (t *LineStringBuilder) ResetState() {
	t.lastNFeatures = []*geojson.Feature{}
	t.intervalFeatures = []*geojson.Feature{}
}

// calculatedAverageSpeedAbsolute returns the average speed in meters per second
// between the first point and last. Round trips will theoretically return 0.
func calculatedAverageSpeedAbsolute(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return 0
	}
	firstTime := mustGetTime(pointFeatures[0], "Time")
	lastTime := mustGetTime(pointFeatures[len(pointFeatures)-1], "Time")
	timeDelta := math.Abs(lastTime.Sub(firstTime).Seconds())
	distance := geo.Distance(pointFeatures[0].Point(), pointFeatures[len(pointFeatures)-1].Point())

	return distance / timeDelta
}

func averageSpeedTraversed(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return 0
	}
	sum := 0.0
	for i := 1; i <= len(pointFeatures)-1; i++ {
		sum += calculatedAverageSpeedAbsolute([]*geojson.Feature{pointFeatures[i], pointFeatures[i-1]})
	}
	return sum / float64(len(pointFeatures))
}

func calculatedAverageSpeedTraversedLinestring(lineString *geojson.Feature) float64 {
	if len(lineString.Geometry.(orb.LineString)) < 2 {
		return 0
	}
	distance := 0.0
	for i := 1; i < len(lineString.Geometry.(orb.LineString)); i++ {
		distance += geo.Distance(lineString.Geometry.(orb.LineString)[i-1], lineString.Geometry.(orb.LineString)[i])
	}
	return distance / lineString.Properties["Duration"].(float64)
}

func averageSpeedReported(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return pointFeatures[0].Properties["Speed"].(float64)
	}
	sum := 0.0
	for _, f := range pointFeatures {
		sum += f.Properties["Speed"].(float64)
	}
	return sum / float64(len(pointFeatures))
}

func timespan(a, b *geojson.Feature) time.Duration {
	if a == nil || b == nil {
		return 0
	}
	firstTime := mustGetTime(a, "Time")
	lastTime := mustGetTime(b, "Time")
	return lastTime.Sub(firstTime)
}

func getAbsoluteDistance(features []*geojson.Feature) float64 {
	if len(features) < 2 {
		return 0
	}
	var start orb.Point
	var end orb.Point
	if _, ok := features[0].Geometry.(orb.Point); ok {
		start = features[0].Point()
	} else {
		start = features[0].Geometry.(orb.LineString)[0]
	}
	if _, ok := features[len(features)-1].Geometry.(orb.Point); ok {
		end = features[len(features)-1].Point()
	} else {
		end = features[len(features)-1].Geometry.(orb.LineString)[len(features[len(features)-1].Geometry.(orb.LineString))-1]
	}
	return geo.Distance(start.Point(), end.Point())
}

func getTraversedDistance(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return 0
	}
	sum := 0.0
	for i := 1; i < len(pointFeatures); i++ {
		sum += getAbsoluteDistance(
			[]*geojson.Feature{
				pointFeatures[i-1], pointFeatures[i],
			})
	}
	return sum
}

func calculatedAverageAccuracy(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) == 0 {
		return 0
	}
	sum := 0.0
	for _, f := range pointFeatures {
		sum += f.Properties["Accuracy"].(float64)
	}
	return sum / float64(len(pointFeatures))
}

func getTraversedElevations(pointFeatures []*geojson.Feature) (up, dn float64) {
	if len(pointFeatures) < 2 {
		return 0, 0
	}
	for i := 1; i < len(pointFeatures); i++ {
		delta := pointFeatures[i].Properties["Elevation"].(float64) - pointFeatures[i-1].Properties["Elevation"].(float64)
		if delta > 0 {
			up += delta
		} else {
			dn += delta
		}
	}
	return up, dn

}

func (t *LineStringBuilder) AddPointFeatureToLastLinestring(f *geojson.Feature) {
	t.LineStringFeature.Geometry = append(t.LineStringFeature.Geometry.(orb.LineString), f.Point())
	t.LineStringFeatures = append(t.LineStringFeatures, f)

	// Atomic (individual) values.
	t.LineStringFeature.Properties["EndTime"] = f.Properties["Time"]
	t.LineStringFeature.Properties["UnixEndTime"] = f.Properties["UnixTime"]
	t.LineStringFeature.Properties["MotionStateReasonEnd"] = f.Properties["MotionStateReason"]

	// Values which require the entire linestring to be calculated.
	durationSeconds := timespan(t.LineStringFeatures[0], f).Round(time.Second).Seconds()
	t.LineStringFeature.Properties["PointCount"] = len(t.LineStringFeatures)
	t.LineStringFeature.Properties["Activity"] = activityModeNotUnknown(t.LineStringFeatures).String()
	t.LineStringFeature.Properties["AverageAccuracy"] = toFixed(calculatedAverageAccuracy(t.LineStringFeatures), 0)
	t.LineStringFeature.Properties["Duration"] = durationSeconds
	t.LineStringFeature.Properties["DistanceAbsolute"] = toFixed(getAbsoluteDistance(t.LineStringFeatures), 2)
	t.LineStringFeature.Properties["AverageReportedSpeed"] = toFixed(averageSpeedReported(t.LineStringFeatures), 2)

	// Incrementally accrued values.
	t.LineStringFeature.Properties["DistanceTraversed"] = toFixed(t.LineStringFeature.Properties["DistanceTraversed"].(float64)+getTraversedDistance(t.LineStringFeatures[len(t.LineStringFeatures)-2:]), 2)
	t.LineStringFeature.Properties["AverageCalculatedSpeed"] = toFixed(t.LineStringFeature.Properties["DistanceTraversed"].(float64)/durationSeconds, 2)

	up, dn := getTraversedElevations(t.LineStringFeatures[len(t.LineStringFeatures)-2:])
	t.LineStringFeature.Properties["ElevationGain"] = toFixed(t.LineStringFeature.Properties["ElevationGain"].(float64)+up, 2)
	t.LineStringFeature.Properties["ElevationLoss"] = toFixed(t.LineStringFeature.Properties["ElevationLoss"].(float64)+dn, 2)
}

func (t *LineStringBuilder) AddPointFeatureToNewLinestring(f *geojson.Feature) {
	// Send the last linestring to the channel.
	if t.LineStringFeature != nil && len(t.LineStringFeature.Geometry.(orb.LineString)) > 1 {
		t.linestringsCh <- t.LineStringFeature
	} else {
		// Otherwise the target linestring will be overwritten with the new feature
		// without sending the last feature.
	}

	t.LineStringFeatures = []*geojson.Feature{f}
	t.LineStringFeature = geojson.NewFeature(orb.LineString{f.Point()})
	t.LineStringFeature.Properties = map[string]interface{}{}
	for k, v := range f.Properties {
		t.LineStringFeature.Properties[k] = v
	}

	t.LineStringFeature.Properties["Name"] = f.Properties["Name"]
	t.LineStringFeature.Properties["UUID"] = f.Properties["UUID"]

	t.LineStringFeature.Properties["StartTime"] = f.Properties["Time"]
	t.LineStringFeature.Properties["Duration"] = 0.0
	t.LineStringFeature.Properties["MotionStateReasonStart"] = f.Properties["MotionStateReason"]
	t.LineStringFeature.Properties["DistanceTraversed"] = 0.0
	t.LineStringFeature.Properties["ElevationGain"] = 0.0
	t.LineStringFeature.Properties["ElevationLoss"] = 0.0
}

var flagLinestringDisconinuityActivities = flag.Bool("linestring-discontinuity-activities", false, "If true, linestrings will be split on activity changes.")

// IsDiscontinuous returns true if the last point is discontinuous from the previous interval.
// According to some paper I found somewhere onetime, it's generally better
// to break more than less. Trips/lines can then be synthesized later.
func (t *LineStringBuilder) IsDiscontinuous(f *geojson.Feature) (isDiscontinuous bool) {
	// A failsafe in case something goes wrong.
	// We assume that t.LastFeature() returns a non-nil value.
	if len(t.lastNFeatures) == 0 {
		return true
	}
	if f == nil {
		return true
	}

	// Split any tracks separated by the DWELL INTERVAL in time,
	// or any non-chronological points.
	span := timespan(t.LastFeature(), f)
	// Use a small buffer to account for floating point errors and time imperfections
	// coming from the cat tracker herself.
	// When we test strictly (< 0), small chrono-overlaps in reported
	// points' reported times will cause over-breaking of linestrings,
	// ie 12:03.02 and 12:03.99. We want to avoid this, and treat
	// these as continuous.
	if span.Seconds() < -1 {
		log.Println("WARN: linestring feature not chronological",
			mustGetTime(t.LastFeature(), "Time"),
			mustGetTime(f, "Time"),
			span.Round(time.Second))
		return true
	}
	if span > *flagDwellInterval {
		return true
	}

	// // If the activity type changes, we should start a new linestring.
	if *flagLinestringDisconinuityActivities {
		incumbent := activityMode(t.intervalFeatures)
		next := activityMode(append(t.intervalFeatures, f))
		if !incumbent.IsContinuous(next) {
			return true
		}
	}

	return false
}

func (t *LineStringBuilder) AddPointFeature(f *geojson.Feature) {
	defer t.AddFeatureToState(f)

	// This will be ONLY the first point the builder sees
	// since the deferred AddFeatureToState will get called for all points.
	if len(t.lastNFeatures) == 0 {
		t.AddPointFeatureToNewLinestring(f)
		return
	}

	if t.IsDiscontinuous(f) {
		t.ResetState()
		t.AddPointFeatureToNewLinestring(f)
		return
	}

	t.AddPointFeatureToLastLinestring(f)
}
