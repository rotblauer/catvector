package main

import (
	"github.com/paulmach/orb"
	"testing"

	"github.com/montanaflynn/stats"
)

func TestWhatIsMode(t *testing.T) {
	floats := []float64{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 2, 2, 1, 1}
	stats := stats.Float64Data(floats)
	mode, err := stats.Mode()
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("Mode: %v", mode)
}

func TestSegmentsIntersect(t *testing.T) {

	type testCase struct {
		segment1  orb.LineString
		segment2  orb.LineString
		intersect bool
	}

	testCases := []testCase{
		{orb.LineString{{0, 0}, {1, 1}}, orb.LineString{{0, 1}, {1, 0}}, true},     // x marks spot
		{orb.LineString{{0, 0}, {-1, -1}}, orb.LineString{{0, -1}, {-1, 0}}, true}, // x marks spot
		{orb.LineString{{0, 0}, {1, 1}}, orb.LineString{{1, 1}, {2, 2}}, false},    // same slope extension
		{orb.LineString{{0, 0}, {1, 1}}, orb.LineString{{1, 1}, {1, 2}}, false},    // different slope extension
		{orb.LineString{{0, 0}, {1, 1}}, orb.LineString{{1, 1}, {0, 0.1}}, false},  // near-reverse angle, no intersection
		{orb.LineString{{0, 0}, {1, 1}}, orb.LineString{{3, 3}, {4, 6}}, false},    // discontinuous, same slope, no intersection
		{orb.LineString{{0, 0}, {0, 1}}, orb.LineString{{0, 1}, {0, 2}}, false},    // vertical, no intersection
		{orb.LineString{{0, 0}, {1, 1}}, orb.LineString{{0, 0}, {-1, -1}}, false},
	}

	for i, tc := range testCases {
		//isIntersection := tc.segment1.Bound().Pad(-segmentBoundEpsilon).Intersects(tc.segment2.Bound().Pad(-segmentBoundEpsilon))
		//if isIntersection != tc.intersect {
		//	t.Errorf("Test case %d failed: expected %v, got %v", i, tc.intersect, isIntersection)
		//}
		isIntersection := segmentsIntersect(tc.segment1, tc.segment2)
		if isIntersection != tc.intersect {
			t.Errorf("Test case %d failed: expected %v, got %v", i, tc.intersect, isIntersection)
		}
	}
}
