package main

import (
	"encoding/json"
	"image/color"
	"io"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"testing"

	sm "github.com/flopp/go-staticmaps"
	"github.com/fogleman/gg"
	"github.com/golang/geo/s2"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/planar"
	"github.com/paulmach/orb/simplify"
)

var testdataRootPath = filepath.Join(".", "testdata")
var testdataOutputRootPath = filepath.Join(testdataRootPath, "output")

func readTestdataFile(geojsonFile string) (io.Reader, error) {
	return os.Open(filepath.Join(testdataRootPath, geojsonFile))
}

func init() {
	flagDwellDistanceThresholdDefault = 50.0
}

func TestCmdConsolidateStops(t *testing.T) {
	// testdata/batch-0010.json.gz
	cmd := exec.Command("zcat", "testdata/batch-0010.json.gz")
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		t.Fatal(err)
	}
	r, w, err := os.Pipe()
	if err != nil {
		t.Fatal(err)
	}
	if err := cmd.Start(); err != nil {
		t.Fatal(err)
	}
	cmdTripDetector(stdout, w)
	cmdConsolidateStops(r, os.Stderr)
	if err := cmd.Wait(); err != nil {
		t.Fatal(err)
	}
}

func TestRKalmanFilter(t *testing.T) {
	in, err := readTestdataFile("edge-rye-tail.json")
	if err != nil {
		t.Fatal(err)
	}

	ctx := sm.NewContext()
	ctx.SetSize(2400, 2400*9/16)
	originalColor := color.RGBA{0, 0, 255, 200}
	kalmanColor := color.RGBA{255, 0, 0, 255}

	filter := &RKalmanFilterT{}
	kFeatureCh, errCh, closeCh := readStreamWithFeatureCallback(in, func(feature *geojson.Feature) (out *geojson.Feature, err error) {
		if filter.Filter == nil {
			if err := filter.InitFromPoint(feature); err != nil {
				t.Fatal(err)
			}
		}

		original := sm.NewCircle(s2.LatLngFromDegrees(feature.Geometry.(orb.Point)[1], feature.Geometry.(orb.Point)[0]), originalColor, originalColor, 1, 0.5)
		ctx.AddObject(original)

		estimate, err := filter.EstimateFromObservation(feature)
		if err != nil {
			return nil, err
		}
		return estimate, nil
	})

loop:
	for {
		select {
		case f := <-kFeatureCh:
			estimated := sm.NewCircle(s2.LatLngFromDegrees(f.Geometry.(orb.Point)[1], f.Geometry.(orb.Point)[0]), kalmanColor, kalmanColor, 1, 0.5)
			ctx.AddObject(estimated)

		case e := <-errCh:
			t.Logf("error: %v", e)
		case <-closeCh:
			break loop
		}
	}

	img, err := ctx.Render()
	if err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(testdataOutputRootPath, 0777); err != nil {
		t.Fatal(err)
	}
	if err := gg.SavePNG(filepath.Join(testdataOutputRootPath, "rye_edge_kalman_filter.png"), img); err != nil {
		t.Fatal(err)
	}
}

func paintMapWriting(f *geojson.Feature, pathto string) error {
	ctx := sm.NewContext()
	ctx.SetSize(800, 800)

	switch g := f.Geometry.(type) {
	case orb.LineString:
		lls := []s2.LatLng{}
		for _, v := range g {
			lls = append(lls, s2.LatLngFromDegrees(v[1], v[0]))
		}
		v := sm.NewPath(lls, color.RGBA{0, 0, 255, 255}, 3)
		ctx.AddObject(v)
	}

	img, err := ctx.Render()
	if err != nil {
		return err
	}

	os.MkdirAll(filepath.Dir(pathto), 0777)
	if err := gg.SavePNG(pathto, img); err != nil {
		return err
	}
	return nil
}

func TestReadStreamToLineString(t *testing.T) {
	in, err := readTestdataFile("edge2.json")
	if err != nil {
		t.Fatal(err)
	}
	f, err := readStreamToLineString(in)
	if err != nil {
		t.Fatal(err)
	}

	// j, err := json.MarshalIndent(f, "", "  ")
	j, err := json.Marshal(f)
	if err != nil {
		t.Fatal(err)
	}
	log.Println(string(j))

	if err := paintMapWriting(f, filepath.Join(testdataOutputRootPath, "edge_original.png")); err != nil {
		t.Fatal(err)
	}

	lsSimpler := f.Geometry.(orb.LineString).Clone()

	// Simplify
	threshold := planar.Distance(orb.Point{0, 0}, orb.Point{0, 0.00006})
	log.Printf("threshold: %f", threshold)
	simplifier := simplify.DouglasPeucker(float64(6) * earthCircumferenceDegreesPerMeter)
	lsSimpler = simplifier.Simplify(lsSimpler).(orb.LineString)

	// j, err := json.MarshalIndent(f, "", "  ")
	j, err = json.Marshal(lsSimpler)
	if err != nil {
		t.Fatal(err)
	}
	log.Println(string(j))

	distance := planar.Distance(f.Geometry.Bound().Min, f.Geometry.Bound().Max)
	t.Logf("distance: %f", distance)
	metersQ := earthCircumference * distance
	t.Logf("metersQ: %f", metersQ)

	if err := paintMapWriting(geojson.NewFeature(lsSimpler), filepath.Join(testdataOutputRootPath, "edge_simplified.png")); err != nil {
		t.Fatal(err)
	}

	// KFilter

	lsKalman := f.Geometry.(orb.LineString).Clone()
	lsKalmanF := geojson.NewFeature(lsKalman)
	for k, v := range f.Properties {
		lsKalmanF.Properties[k] = v
	}
	if err := modifyLineStringKalman(lsKalmanF); err != nil {
		t.Fatal(err)
	}

	if err := paintMapWriting(lsKalmanF, filepath.Join(testdataOutputRootPath, "edge_kalman.png")); err != nil {
		t.Fatal(err)
	}

	// KFilter - github.com/Regnull - geokalmanfilter

	lsKalmanRegnull := f.Geometry.(orb.LineString).Clone()
	lsKalmanRegnullF := geojson.NewFeature(lsKalmanRegnull)
	for k, v := range f.Properties {
		lsKalmanRegnullF.Properties[k] = v
	}
	if err := modifyLineStringKalmanRegnull(lsKalmanRegnullF); err != nil {
		t.Fatal(err)
	}

	if err := paintMapWriting(lsKalmanRegnullF, filepath.Join(testdataOutputRootPath, "edge_kalman_regnull.png")); err != nil {
		t.Fatal(err)
	}

}

func TestToFixed(t *testing.T) {
	type testT struct {
		initial   float64
		precision int
		want      float64
	}
	for _, test := range []testT{
		{0.123456789, 0, 0},
		{123456789, 0, 123456789},
		{0.123456789, 2, 0.12},
		{0.123456789, 4, 0.1235},
		{0.123456789, 6, 0.123457},
		{0.123456789, 8, 0.12345679},
		{200.123456789, 2, 200.12},
		{20000.123456789, 4, 20000.1235},
	} {
		got := toFixed(test.initial, test.precision)
		if got != test.want {
			t.Errorf("got: %v, want: %v", got, test.want)
		}
	}
}
