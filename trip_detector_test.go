package main

import (
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
