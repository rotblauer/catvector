package main

import (
	"fmt"
	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb/planar"
	"math"
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
)

const segmentBoundEpsilon = 1e-5

// TripDetector normally takes Points and turns them into either a Trip or a Stop.
// It is a state machine that takes in a series of points and returns a series of Trips and Stops.
// Trips are defined as a series of points that are moving, structured as LineStrings.
// Stops are defined as a series of points that are stationary, structured as Points.
type TripDetector struct {
	DwellTime      time.Duration
	TripStartTime  time.Duration
	SpeedThreshold float64
	DwellDistance  float64
	lastNPoints    TracksGeoJSON
	intervalPoints TracksGeoJSON
	Tripping       bool

	// MotionStateReason is a string that describes the reason for the current state of the TripDetector.
	MotionStateReason string

	// intervalPtsCentroid is the centroid of the last dwell interval points.
	intervalPtsCentroid orb.Point

	segmentIntersectionGauge float64
}

func NewTripDetector(dwellTime, tripStartTime time.Duration, speedThreshold, dwellDistance float64) *TripDetector {
	return &TripDetector{
		DwellTime:      dwellTime,
		TripStartTime:  tripStartTime,
		SpeedThreshold: speedThreshold,
		DwellDistance:  dwellDistance,
		Tripping:       false,
		lastNPoints:    TracksGeoJSON{},
		intervalPoints: TracksGeoJSON{},
	}
}

func (d *TripDetector) LastPointN(n int) *TrackGeoJSON {
	if len(d.lastNPoints) == 0 {
		return nil
	}
	return d.lastNPoints[len(d.lastNPoints)-1-n]
}

func (d *TripDetector) IntervalPointsWhere(include func(t *TrackGeoJSON) bool) []*geojson.Feature {
	out := []*geojson.Feature{}
	for i := range d.intervalPoints {
		if include(d.intervalPoints[i]) {
			out = append(out, d.intervalPoints[i].Feature)
		}
	}
	return out
}

func (d *TripDetector) AddFeatureToState(f *geojson.Feature) {
	d.intervalPoints = append(d.intervalPoints, &TrackGeoJSON{f})
	for i := len(d.intervalPoints) - 1; i > 0; i-- {
		// Use a double-size window to hold the interval points.
		// We'll want to bound certain measurements on the dwell time (the interval),
		// so we want some wiggle room in there.
		if d.intervalPoints.Timespan() > d.DwellTime*2 {
			d.intervalPoints = d.intervalPoints[i:]
			break
		}
	}

	d.lastNPoints = append(d.lastNPoints, &TrackGeoJSON{f})
	for len(d.lastNPoints) > 10 {
		d.lastNPoints = d.lastNPoints[1:]
	}

	// Degrade the segment intersection gauge.
	n := float64(0)
	d.intervalPoints.ForEach(func(t *TrackGeoJSON) {
		if t.MustGetTime().Before(t.MustGetTime().Add(-d.DwellTime)) {
			return
		}
		n += 1.0
	})
	d.segmentIntersectionGauge *= 1 - (1.0 / n)
}

func (d *TripDetector) ResetState() {
	d.Tripping = false
	d.MotionStateReason = "reset"
	d.intervalPtsCentroid = orb.Point{}
	d.lastNPoints = TracksGeoJSON{}
	d.intervalPoints = TracksGeoJSON{}
	d.segmentIntersectionGauge = 0
}

// segmentsIntersect returns true if the two line segments intersect.

func segmentsIntersect(segA, segB orb.LineString) (intersect bool, x, y *float64) {

	/*
		// https://www.topcoder.com/thrive/articles/Geometry%20Concepts%20part%202:%20%20Line%20Intersection%20and%20its%20Applications#LineLineIntersection

		aMax, aMin := segA.Bound().Max, segA.Bound().Min
		a1 := aMax.Y() - aMin.Y()
		b1 := aMin.X() - aMax.X()
		c1 := a1*aMin.X() + b1*aMin.Y()

		bMax, bMin := segB.Bound().Max, segB.Bound().Min
		a2 := bMax.Y() - bMin.Y()
		b2 := bMin.X() - bMax.X()
		c2 := a2*bMin.X() + b2*bMin.Y()

		det := a1*b2 - a2*b1
		if det == 0 {
			return false // parallel
		}

		// Derive the intersection point of the LINES.
		iX := (b2*c1 - b1*c2) / det
		iY := (a1*c2 - a2*c1) / det

		// Return true if the intersection point of the LINES is within the BOUNDS of the SEGMENTS.
		//return iX >= aMin.X() && iX <= aMax.X() && iY >= aMin.Y() && iY <= aMax.Y()
		//return iX > aMin.X() && iX < aMax.X() && iY > aMin.Y() && iY < aMax.Y() // strict intersection (not at endpoints)
		return iX < math.Max(aMin.X(), aMax.X()) && iX > math.Min(aMin.X(), aMax.X()) && iY < math.Max(aMin.Y(), aMax.Y()) && iY > math.Min(aMin.Y(), aMax.Y())
	*/
	/*
		https://stackoverflow.com/a/1968345

		// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
		// intersect the intersection point may be stored in the floats i_x and i_y.
		char get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
		    float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
		{
		    float s1_x, s1_y, s2_x, s2_y;
		    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
		    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

		    float s, t;
		    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
		    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

		    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		    {
		        // Collision detected
		        if (i_x != NULL)
		            *i_x = p0_x + (t * s1_x);
		        if (i_y != NULL)
		            *i_y = p0_y + (t * s1_y);
		        return 1;
		    }

		    return 0; // No collision
		}
	*/
	p0_x, p0_y := segA[0][0], segA[0][1]
	p1_x, p1_y := segA[1][0], segA[1][1]
	p2_x, p2_y := segB[0][0], segB[0][1]
	p3_x, p3_y := segB[1][0], segB[1][1]
	var s1_x, s1_y, s2_x, s2_y float64
	s1_x = p1_x - p0_x
	s1_y = p1_y - p0_y
	s2_x = p3_x - p2_x
	s2_y = p3_y - p2_y
	var s, t float64
	s = (-s1_y*(p0_x-p2_x) + s1_x*(p0_y-p2_y)) / (-s2_x*s1_y + s1_x*s2_y)
	t = (s2_x*(p0_y-p2_y) - s2_y*(p0_x-p2_x)) / (-s2_x*s1_y + s1_x*s2_y)
	if s >= 0 && s <= 1 && t >= 0 && t <= 1 {
		i_x := p0_x + (t * s1_x)
		i_y := p0_y + (t * s1_y)
		i := orb.Point{i_x, i_y}
		// IsDetectStopIntersection is considered exclusive to the endpoints of a (either) segment.
		// My use case is primarily contiguous segments.
		if !segA.Bound().Min.Equal(i) && !segA.Bound().Max.Equal(i) {
			return true, &i_x, &i_y
		}
	}
	return false, nil, nil
}

// AddFeature takes a geojson.Feature and adds it to the state of the TripDetector.
// The TripDetector will then decide if the feature determines a TripDetector state of Tripping or not,
// and will update its state accordingly.
func (d *TripDetector) AddFeature(f *geojson.Feature) error {
	t := &TrackGeoJSON{f}

	defer d.AddFeatureToState(f)

	last := d.LastPointN(0)
	if last != nil {
		// Ensure chronology or reset and return.
		if last.MustGetTime().After(t.MustGetTime()) {
			d.ResetState()
			return nil
		}
	} else {
		// Last was nil, so we are starting over.
		return nil
	}

	// From here, last is assuredly non-nil.

	/*
			Identifying trip ends with signal loss.

			The dwell time is most frequently used in the existing researches to infer
			trip ends with signal loss. If the time difference between
			two consecutive GPS points exceeds a certain threshold,
		 	we suppose that a potential trip end will occur.
			Based on the previous studies, 120 s is usually employed
			to represent the minimum time gap that an activity
			would reasonably take place. We select GPS records
			with time difference for more than 120 s as the potential
			trip ends. As has been mentioned before, signal loss
			generally occurs due to the signal blocking when volunteers
			are in the indoor buildings or underground. To
			remove the pseudo trip ends, we compare the average
			speed of the signal loss segment (equal to the distance
			traveled divided by time length of the signal loss period)
			with the lower bound of walking with 0.5 m/s. If the
			average speed of the signal loss segment is less than this
			value, then a real trip end is flagged, while if not, we
			consider it as the pseudo one and remove it.
	*/
	if dwell := t.MustGetTime().Sub(last.MustGetTime()); dwell > d.DwellTime {

		// To remove pseudo trip ends...
		distance := geo.Distance(t.Point(), last.Point())
		speed := distance / dwell.Seconds() // m/s
		if speed < d.SpeedThreshold {
			// Real trip end flagged.
			d.Tripping = false
			d.MotionStateReason = "signal loss"
			return nil
		}
	}

	weight := detectedNeutral
	idPC := d.IsDetectStopPointClustering(f)
	idPCC := d.IsDetectStopPointClusteringCentroid(f)
	idX := d.IsDetectStopIntersection(f)
	idRS := d.IsDetectStopReportedSpeeds(f)
	idO := d.IsDetectStopOverlaps(f)

	d.MotionStateReason = fmt.Sprintf("idPC: %v, idPCC: %v, idX: %v, idRS: %v, idO: %v", idPC, idPCC, idX, idRS, idO)

	weight += idPC
	weight += idPCC
	weight += idX
	weight += idRS
	weight += idO

	// TODO: tinker
	if weight < detectedStop {
		d.Tripping = false
		return nil
	}
	if weight > detectedTrip {
		d.Tripping = true
		return nil
	}

	/*
		d.Tripping = true
		d.MotionStateReason = "default"
	*/

	//if d.IsDetectStopPointClusteringCentroid(f) {
	//	return nil
	//}

	/*
		The above methods are used to identify the trip ends.
		But how do we know when the trip starts?
	*/

	// links := []*geojson.Feature{}
	// for i := len(d.intervalPoints); i > 0; i-- {
	// 	var cur, ref *geojson.Feature
	// 	ref = d.intervalPoints[i-1].Feature
	// 	if i == len(d.intervalPoints) {
	// 		cur = f
	// 	} else {
	// 		cur = d.intervalPoints[i].Feature
	// 	}
	// 	tt := &TrackGeoJSON{cur}
	// 	if tt.MustGetTime().Before(dwellStartMin) {
	// 		break
	// 	}
	// 	link := geojson.NewFeature(orb.LineString{ref.Point(), cur.Point()})
	// 	links = append(links, link)
	// }
	// for i := 0; i < len(links); i++ {
	// 	if i == 0 {
	// 		continue
	// 	}
	// 	linkI := links[i]
	// 	for j := 0; j < i; j++ {
	// 		linkJ := links[j]
	// 		bi := linkI.Geometry.(orb.LineString).Bound()
	// 		bj := linkJ.Geometry.(orb.LineString).Bound()
	// 		if bi.Intersects(bj) {
	//
	// 		}
	// 	}
	// 	overlap := geo.LineOverlap(links[i-1].Geometry.(orb.LineString), links[i].Geometry.(orb.LineString))
	// 	if overlap > 50 {
	// 		d.Tripping = false
	// 		return nil
	// 	}
	// }

	// If we are here, we are UNDECIDED.
	// The TripDetector maintains its state unchanged.

	return nil
}

type detectedT float64

const (
	detectedStop    detectedT = -1
	detectedNeutral detectedT = 0
	detectedTrip    detectedT = 1
)

// IsDetectStopPointClustering is a method that identifies trip ends during normal GPS recording.
/*
	Identifying trip ends during normal GPS recording...

	During
	the normal GPS recording, every point is recorded
	chronologically. Trip ends usually perform with the
	point clustering, where sequential GPS points close to
	each other are in an approximate circle area. To infer
	this type of trip ends, we adopt k-means clustering algorithm
	by calculating the maximum distance between
	any two points in the cluster. We define the diameter of
	10 m of the circular cluster. If the maximum distance
	does not exceed this value, the whole cluster will be
	detected as a potential trip end.

	The first point in the
	cluster in the order of time is the starting of the trip end
	and the last point is the terminal of the trip end. In this
	situation, the dwell time also indicates the minimum
	duration that a real activity should occur. A proper
	dwell time should significantly distinguish real trip ends
	from pseudo ones such as waiting for the traffic signal
	or greeting the acquaintance during the trip. Based on
	the specific traffic situations in Shanghai, we assume
	that a vehicle should be less likely to remain absolutely
	stationary for a traffic signal or traffic congestion for
	more than 120 s. Therefore, we take the dwell time of
	120 s to remove the pseudo trip ends. It is assumed that
	there does exist a trip end if the duration of the point
	clustering exceeds 120 s; otherwise, it is treated as the
	pseudo one and will be removed.
*/
func (d *TripDetector) IsDetectStopPointClustering(f *geojson.Feature) (result detectedT) {
	t := &TrackGeoJSON{f}

	currentTime := t.MustGetTime()
	dwellStartMin := currentTime.Add(-d.DwellTime)

	maxDist := 0.0
	dwellExceeded := false
outer:
	for i := len(d.intervalPoints) - 1; i >= 0; i-- {
		p := d.intervalPoints[i]
		if p.MustGetTime().Before(dwellStartMin) {
			dwellExceeded = true
			break
		}
		dist := geo.Distance(p.Point(), t.Point())
		if dist > maxDist {
			maxDist = dist
		}
		if maxDist > d.DwellDistance {
			// If we ever (within the dwell window) exceed the stop cluster distance, we are done.
			// The trip is not yet stopped.
			break outer
		}
	}
	//speeds := d.intervalPoints.ReportedSpeeds(t.MustGetTime().Add(-d.DwellTime))
	//reportedMeanSpeeds, _ := stats.Mean(stats.Float64Data(speeds))
	if dwellExceeded && maxDist <= d.DwellDistance /* && reportedMeanSpeeds < d.SpeedThreshold */ {
		return detectedStop
	}
	return detectedTrip
}

func (d *TripDetector) IsDetectStopPointClusteringCentroid(f *geojson.Feature) (result detectedT) {
	t := &TrackGeoJSON{f}

	dwellExceeded := false

	// Update the d.intervalPtsCentroid value to reflect the centroid of d.intervalPoints.
	pts := []orb.Point{t.Point()}
	// traverse intervalPoints backwards.
	for i := len(d.intervalPoints) - 1; i >= 0; i-- {
		p := d.intervalPoints[i]
		if p.MustGetTime().Before(t.MustGetTime().Add(-d.DwellTime)) {
			dwellExceeded = true
			break
		}
		pts = append(pts, p.Point())
	}
	d.intervalPtsCentroid, _ = planar.CentroidArea(orb.MultiPoint(pts))

	dist := geo.Distance(d.intervalPtsCentroid, t.Point())
	if dist > d.DwellDistance {
		return detectedTrip
	} else if dwellExceeded {
		return detectedStop
	}
	return detectedNeutral
}

// IsDetectStopIntersection is a method that identifies trip ends with track point segment intersections.
/*
	Experimental: identifying trip ends with track point segment intersections.
	When knots are introduced to our lines, interpret this as a trip end.
*/
func (d *TripDetector) IsDetectStopIntersection(f *geojson.Feature) (result detectedT) {
	t := &TrackGeoJSON{f}

	// Experimental: identifying trip ends with track point segment intersections.
	// When knots are introduced to our lines, interpret this as a trip end.
	dwellIntervalPts := d.IntervalPointsWhere(func(tt *TrackGeoJSON) bool {
		return tt.MustGetTime().After(t.MustGetTime().Add(-d.DwellTime))
	})

	if len(dwellIntervalPts) > 0 {
		currentSegment := orb.LineString{
			dwellIntervalPts[len(dwellIntervalPts)-1].Point(),
			t.Point(),
		}

		for i := len(dwellIntervalPts) - 1; i > 0; i-- {
			segment := orb.LineString{dwellIntervalPts[i-1].Point(), dwellIntervalPts[i].Point()}
			if x, _, _ := segmentsIntersect(segment, currentSegment); x {
				d.segmentIntersectionGauge += 0.025
				//break
			}
		}
	}

	return detectedT(d.segmentIntersectionGauge * float64(detectedStop))
}

// IsDetectStopOverlaps is a method that identifies trip ends with track point segment overlaps.
/*
	In addition, some short trip ends may take less than
	2 min such as ‘‘picking up or dropping off somebody.’’
	Most existing researches identify this type of trip end by
	examining the change in direction to determine whether
	there exists a trip end. However, only considering the
	change in direction may misidentify turning at the inter-
	sections as the trip ends. Actually, drivers usually take
	the same road links before and after picking up/drop-
	ping off somebody. Thus, we calculate the length of
	overlapped links before and after an abrupt change in
	direction. If the overlapped length exceeds the value of
	50 m (considering the physical size of intersections in
	Shanghai), a trip end is flagged.
*/
func (d *TripDetector) IsDetectStopOverlaps(f *geojson.Feature) (result detectedT) {
	t := &TrackGeoJSON{f}

	// Experimental: identifying trip ends with track point segment intersections.
	// When knots are introduced to our lines, interpret this as a trip end.
	dwellIntervalPts := d.IntervalPointsWhere(func(tt *TrackGeoJSON) bool {
		return tt.MustGetTime().After(t.MustGetTime().Add(-d.DwellTime))
	})

	if len(dwellIntervalPts) > 0 {
		currentSegment := orb.LineString{
			dwellIntervalPts[len(dwellIntervalPts)-1].Point(),
			t.Point(),
		}

		for i := len(dwellIntervalPts) - 1; i > 0; i-- {
			segment := orb.LineString{dwellIntervalPts[i-1].Point(), dwellIntervalPts[i].Point()}

			// I want the length of the segment on the linestring between the intersection.
			// It's a ring.
			ringLen := 0.0
			if x, _, _ := segmentsIntersect(segment, currentSegment); x {

				// We were stepping backwards through the dwell-interval points.
				// Now we're going to walk foraward through it, since we know
				// that the points are chronological and all points between 'then' and 'now'
				// are considered part of a self-overlapping segment; a knot.
				// We want to know: How long is the loop in the knot?
				for j := i; j < len(dwellIntervalPts); j++ {
					ringLen += geo.Distance(dwellIntervalPts[j-1].Point(), dwellIntervalPts[j].Point())
					// Note that this overcounts the length by the distance between the intersection and the first point
					// and the distance from the intersection and the last point,
					// which is probably about the length of an average segment.
					// Since I'm working in the context of about 120s dwell intervals
					// and 1pt/second GPS records, this seems fine.
				}

				// We weight the documented 50 meter standard by the length of the ring.
				// A 50m ring returns '-1', but larger rings will not return greater values.
				return detectedT(math.Min(float64(ringLen)*0.02, 1.0)) * detectedStop
			}
		}
	}

	return detectedNeutral
}

func (d *TripDetector) IsDetectStopReportedSpeeds(f *geojson.Feature) (result detectedT) {
	t := &TrackGeoJSON{f}

	referenceSpeeds := d.intervalPoints.ReportedSpeeds(t.MustGetTime().Add(-d.TripStartTime))
	if len(referenceSpeeds) == 0 {
		return detectedNeutral
	}
	// Get the statistics for this range.
	referenceStats := stats.Float64Data(referenceSpeeds)
	mean, _ := referenceStats.Mean()
	median, _ := referenceStats.Median()
	if mean > d.SpeedThreshold {
		return detectedTrip
	}
	if median < d.SpeedThreshold {
		return detectedStop
	}
	return detectedNeutral
}
