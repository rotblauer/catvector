package main

import (
	"time"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/planar"
)

// TripDetector normally takes Points and turns them into either a Trip or a Stop.
// It is a state machine that takes in a series of points and returns a series of Trips and Stops.
// Trips are defined as a series of points that are moving, structured as LineStrings.
// Stops are defined as a series of points that are stationary, structured as Points.
type TripDetector struct {
	DwellTime           time.Duration
	TripStartTime       time.Duration
	SpeedThreshold      float64
	StopClusterDistance float64
	lastNPoints         TracksGeoJSON
	intervalPoints      TracksGeoJSON
	Tripping            bool
	MotionStateReason   string // Why tripping was tripped or un-tripped.
	StoppedLoc          orb.Point
}

func NewTripDetector(dwellTime, tripStartTime time.Duration, speedThreshold, stopClusterDistance float64) *TripDetector {
	return &TripDetector{
		DwellTime:           dwellTime,
		TripStartTime:       tripStartTime,
		SpeedThreshold:      speedThreshold,
		StopClusterDistance: stopClusterDistance,
		Tripping:            false,
		lastNPoints:         TracksGeoJSON{},
		intervalPoints:      TracksGeoJSON{},
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
}

func (d *TripDetector) ResetState() {
	d.Tripping = false
	d.MotionStateReason = "reset"
	d.StoppedLoc = orb.Point{}
	d.lastNPoints = TracksGeoJSON{}
	d.intervalPoints = TracksGeoJSON{}
}

// AddFeature takes a geojson.Feature and adds it to the state of the TripDetector.
// The TripDetector will then decide if the feature determines a TripDetector state of Tripping or not,
// and will update its state accordingly.
func (d *TripDetector) AddFeature(f *geojson.Feature) error {
	t := &TrackGeoJSON{f}

	defer func() {
		if !d.Tripping {
			// Update the d.StoppedLoc value to reflect the centroid of d.intervalPoints.
			pts := []orb.Point{}
			for _, p := range d.intervalPoints {
				if p.MustGetTime().Before(t.MustGetTime().Add(-d.DwellTime)) {
					break
				}
				pts = append(pts, p.Point())
			}
			d.StoppedLoc, _ = planar.CentroidArea(orb.MultiPoint(pts))
		}
	}()
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

	currentTime := t.MustGetTime()
	dwellStartMin := currentTime.Add(-d.DwellTime)

	maxDist := 0.0
	dwellExceeded := false
	pointsMemo := []*geojson.Feature{f}
outer:
	for i := len(d.intervalPoints) - 1; i >= 0; i-- {
		p := d.intervalPoints[i]
		if p.MustGetTime().Before(dwellStartMin) {
			dwellExceeded = true
			break
		}
		for _, pp := range pointsMemo {
			dist := geo.Distance(p.Point(), pp.Point())
			if dist > maxDist {
				maxDist = dist
			}
			if maxDist > d.StopClusterDistance {
				// If we ever (within the dwell window) exceed the stop cluster distance, we are done.
				// The trip is not yet stopped.
				break outer
			}
		}
	}
	if dwellExceeded && maxDist <= d.StopClusterDistance {
		d.Tripping = false
		d.MotionStateReason = "point clustering"
		return nil
	}

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

		TODO
	*/

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

	// // Get the average reported speed from all the points in the interval.
	// // Get the average geo-calculated speed from all the points in the interval.
	// // Weight the reported speed 80% and the calculated speed 20%.
	referenceSpeeds := d.intervalPoints.ReportedSpeeds(t.MustGetTime().Add(-d.TripStartTime))
	if len(referenceSpeeds) == 0 {
		return nil
	}
	// Get the statistics for this range.
	referenceStats := stats.Float64Data(referenceSpeeds)
	mean, err := referenceStats.Mean()
	if err != nil {
		return err
	}
	if mean > d.SpeedThreshold {
		d.Tripping = true
		d.MotionStateReason = "reported speeds"
		return nil
	}

	// If we are here, we are UNDECIDED.
	// The TripDetector maintains its state unchanged.

	return nil
}
