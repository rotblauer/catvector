package main

import (
	"regexp"

	"github.com/montanaflynn/stats"
	"github.com/paulmach/orb/geojson"
)

type Activity int

const (
	TrackerStateUnknown Activity = iota
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

func (a Activity) IsMoving() bool {
	return a > TrackerStateStationary
}

func (a Activity) String() string {
	switch a {
	case TrackerStateUnknown:
		return "Unknown"
	case TrackerStateStationary:
		return "Stationary"
	case TrackerStateWalking:
		return "Walking"
	case TrackerStateRunning:
		return "Running"
	case TrackerStateCycling:
		return "Bike"
	case TrackerStateDriving:
		return "Automotive"
	}
	return "Unknown"
}

func (a Activity) IsContinuous(b Activity) bool {
	if a == TrackerStateUnknown || b == TrackerStateUnknown {
		return true
	}
	if a == TrackerStateStationary && b >= TrackerStateWalking {
		return false
	}
	if a >= TrackerStateWalking && b == TrackerStateStationary {
		return false
	}
	return true
}

func ActivityFromReport(report interface{}) Activity {
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

func activityMode(list []*geojson.Feature) Activity {
	activities := make([]float64, len(list))
	for i, f := range list {
		act := ActivityFromReport(f.Properties["Activity"])
		activities[i] = float64(act)
	}
	activitiesStats := stats.Float64Data(activities)
	mode, _ := activitiesStats.Mode()
	if len(mode) == 0 {
		return TrackerStateUnknown
	}
	return Activity(mode[0])
}

func activityModeNotUnknown(list []*geojson.Feature) Activity {
	activities := []float64{}
	for _, f := range list {
		act := ActivityFromReport(f.Properties["Activity"])
		if act > TrackerStateUnknown {
			activities = append(activities, float64(act))
		}
	}
	activitiesStats := stats.Float64Data(activities)
	mode, _ := activitiesStats.Mode()
	for _, m := range mode {
		if m != float64(TrackerStateUnknown) {
			return Activity(m)
		}
	}
	return TrackerStateUnknown
}
