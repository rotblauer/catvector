package main

import (
	"log"
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
)

type PointTracker struct {
	Interval           time.Duration
	LineStringFeature  *geojson.Feature
	LineStringFeatures []*geojson.Feature // the points represented by the linestring
	lastNFeatures      []*geojson.Feature
	intervalFeatures   []*geojson.Feature // points
	linestringsCh      chan *geojson.Feature
}

func NewPointTracker(interval time.Duration) *PointTracker {
	return &PointTracker{
		Interval:         interval,
		lastNFeatures:    make([]*geojson.Feature, 0),
		intervalFeatures: make([]*geojson.Feature, 0),
		linestringsCh:    make(chan *geojson.Feature),
	}
}

func (t *PointTracker) LastFeature() *geojson.Feature {
	if len(t.lastNFeatures) == 0 {
		return nil
	}
	return t.lastNFeatures[len(t.lastNFeatures)-1]
}

func (t *PointTracker) Flush() {
	if t.LineStringFeature != nil {
		t.linestringsCh <- t.LineStringFeature
	}
	close(t.linestringsCh)
}

func (t *PointTracker) AddFeatureToState(f *geojson.Feature) {
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

func (t *PointTracker) ResetState() {
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
	timeDelta := lastTime.Sub(firstTime).Seconds()
	distance := geo.Distance(pointFeatures[0].Point(), pointFeatures[len(pointFeatures)-1].Point())

	return distance / timeDelta
}

func calculatedAverageSpeedTraversed(pointFeatures []*geojson.Feature) float64 {
	if len(pointFeatures) < 2 {
		return 0
	}
	sum := 0.0
	for i := 1; i < len(pointFeatures); i++ {
		sum += calculatedAverageSpeedAbsolute(pointFeatures[i-1 : i+1])
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
		sum += getAbsoluteDistance(pointFeatures[i-1 : i+1])
	}
	return sum
}

func getActivityTypeForFeature(f *geojson.Feature) Activity {
	if f.Properties["Activity"] == nil {
		return TrackerStateUnknown
	}
	return ActivityFromReport(f.Properties["Activity"])
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

func (t *PointTracker) AddPointFeatureToLastLinestring(f *geojson.Feature) {
	t.LineStringFeature.Geometry = append(t.LineStringFeature.Geometry.(orb.LineString), f.Point())
	t.LineStringFeatures = append(t.LineStringFeatures, f)

	// TODO: update properties properly
	for k, v := range f.Properties {
		t.LineStringFeature.Properties[k] = v
	}

	t.LineStringFeature.Properties["NumberOfPoints"] = len(t.LineStringFeatures)
	t.LineStringFeature.Properties["Activity"] = activityModeNotUnknown(t.LineStringFeatures).String()
	t.LineStringFeature.Properties["AverageAccuracy"] = calculatedAverageAccuracy(t.LineStringFeatures)
	t.LineStringFeature.Properties["Duration"] = timespan(t.LineStringFeatures[0], f).Round(time.Second).Seconds()
	t.LineStringFeature.Properties["DistanceTraversed"] = getTraversedDistance(t.LineStringFeatures)
	t.LineStringFeature.Properties["DistanceAbsolute"] = getAbsoluteDistance(t.LineStringFeatures)
	t.LineStringFeature.Properties["AverageReportedSpeed"] = averageReportedSpeed(t.LineStringFeatures)
	t.LineStringFeature.Properties["AverageCalculatedSpeed"] = calculatedAverageSpeedTraversed(t.LineStringFeatures)
}

func joinLinestrings(a, b *geojson.Feature) *geojson.Feature {
	ls := orb.LineString{}
	ls = append(ls, a.Geometry.(orb.LineString)...)
	ls = append(ls, b.Geometry.(orb.LineString)...)
	joined := geojson.NewFeature(ls)
	joined.Properties = map[string]interface{}{}
	for k, v := range a.Properties {
		joined.Properties[k] = v
	}
	for k, v := range b.Properties {
		joined.Properties[k] = v
	}
	joined.Properties["StartTime"] = a.Properties["StartTime"]
	joined.Properties["Duration"] = a.Properties["Duration"].(float64) + b.Properties["Duration"].(float64)
	joined.Properties["DistanceTraversed"] = a.Properties["DistanceTraversed"].(float64) + b.Properties["DistanceTraversed"].(float64)
	joined.Properties["DistanceAbsolute"] = getAbsoluteDistance([]*geojson.Feature{a, b})
	// For now, just return a weighted average of reported speeds.
	joined.Properties["AverageReportedSpeed"] = (a.Properties["AverageReportedSpeed"].(float64)*a.Properties["NumberofPoints"].(float64) +
		b.Properties["AverageReportedSpeed"].(float64)*b.Properties["NumberOfPoints"].(float64)) /
		(a.Properties["NumberofPoints"].(float64) + b.Properties["NumberofPoints"].(float64))

	joined.Properties["AverageCalculatedSpeed"] = calculatedAverageSpeedTraversedLinestring(joined)

	return joined
}

func (t *PointTracker) AddPointFeatureToNewLinestring(f *geojson.Feature) {
	// Send the last linestring to the channel.
	if t.LineStringFeature != nil && len(t.LineStringFeature.Geometry.(orb.LineString)) > 1 {
		t.linestringsCh <- t.LineStringFeature
	}

	t.LineStringFeatures = []*geojson.Feature{f}
	t.LineStringFeature = geojson.NewFeature(orb.LineString{f.Point()})
	t.LineStringFeature.Properties = map[string]interface{}{}
	for k, v := range f.Properties {
		t.LineStringFeature.Properties[k] = v
	}
	t.LineStringFeature.Properties["StartTime"] = f.Properties["Time"]
	t.LineStringFeature.Properties["Duration"] = 0.0
}

// IsDiscontinuous returns true if the last point is discontinuous from the previous interval.
// According to some paper I found somewhere onetime, it's generally better
// to break more than less. Trips/lines can then be synthesized later.
func (t *PointTracker) IsDiscontinuous(f *geojson.Feature) (isDiscontinuous bool) {
	if len(t.lastNFeatures) == 0 {
		return true
	}

	currentTime := mustGetTime(f, "Time")
	lastTime := mustGetTime(t.LastFeature(), "Time")
	if currentTime.Before(lastTime) {
		// Reset
		log.Println("WARN: feature not chronological, resetting tracker", currentTime, lastTime, currentTime.Sub(lastTime).Round(time.Second))
		t.ResetState()
		return true
	}

	if len(t.lastNFeatures) == 1 {
		return false
	}

	// Split any tracks separated by the INTERVAL in time.
	if span := timespan(t.LastFeature(), f); span > *flagDwellInterval || span < 0 {
		return true
	}
	
	// // If the activity type changes, we should start a new linestring.
	// incumbent := activityMode(t.intervalFeatures)
	// next := activityMode(append(t.intervalFeatures, f))
	// if !incumbent.IsContinuous(next) {
	// 	return true
	// }

	return false
}

func (t *PointTracker) AddPointFeature(f *geojson.Feature) {
	if t.IsDiscontinuous(f) {
		t.AddPointFeatureToNewLinestring(f)
	} else {
		t.AddPointFeatureToLastLinestring(f)
	}
	t.AddFeatureToState(f)
}
